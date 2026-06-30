#pragma once

#include <atomic>
#include <deque>
#include <set>
#include <thread>
#include <utility>
#include "FrameCadence.h"
#include "Utils.hpp"
#include "VideoRenderer.h"

extern "C" {
#include <libavcodec/avcodec.h>
}

class Pacer {
  public:
	// Singleton accessor
	static Pacer &instance();

	// Frame pacing strategy (selectable from the quick menu). DISPLAY_LOCKED routes
	// through renderModeDisplayLocked; the rest are renderModeImmediate variants that
	// differ only in their drop policy.
	enum PacingMode {
		PACING_DISPLAY_LOCKED = 0,  // present every vblank, cadence-managed repetition (smoother, +1 frame)
		PACING_DRAIN = 1,           // immediate: render newest, drop older every present (lowest latency)
		PACING_QT = 2,              // immediate: moonlight-qt hysteresis (drop only on a persistent backlog)
		PACING_ADAPTIVE = 3,        // immediate: buffer target sized to measured arrival jitter
		PACING_LEGACY = 4,          // immediate: original upstream off-by-one (for A/B against the bug)
		PACING_MODE_COUNT = 5,      // for cycling
	};

	void deinit();
	void init(const std::shared_ptr<DX::DeviceResources> &res, int maxVideoFps, double refreshRate, bool framePacingImmediate);
	bool getPacingImmediate();              // = (mode != DISPLAY_LOCKED); kept for stats/compat
	void setPacingImmediate(bool framePacingImmediate);  // DRAIN <-> DISPLAY_LOCKED (config init + #253 toggle)
	int getPacingMode();
	void setPacingMode(int mode);
	int getAdaptiveTarget();                // current PACING_ADAPTIVE drop target (stats/overlay)
	double getRecentLossPressure();         // decaying producer-side loss count (tuner's clean/loss signal)
	void loadTuningParams();                // re-read adaptive constants from LocalState (no rebuild)
	void resetTraceLogs();                  // clear the CSV pacing trace (debug)
	void recomputeWeights();                // debug: fit p1/p2/p3 from per-condition stats + apply
	// Scene class for the auto-tuner. Auto-detected from the loss breakdown (Stats), not
	// labelled by hand: network loss vs decoder can't-keep-up vs clean.
	enum Condition { COND_CLEAN = 0, COND_PACING = 1, COND_NETWORK = 2, COND_COUNT = 3 };
	// Current-condition objective readout for the overlay (debug auto-tuner).
	struct TuneView {
		int    cond;          // detected scene class
		double samples;       // presents accumulated in this scene since the last reset
		double lossPer1k;     // producer-side lost-frame events per 1000 presents
		int    maxBurst;      // largest loss burst seen (frames) -> drives the needed depth
		int    neededDepth;   // buffer depth this scene needs (1 + maxBurst, +safety)
		double avgTarget;     // mean buffer depth the controller actually used
		double recentBurst;   // decaying max of arrivals/present-interval (primary growth signal)
		int    arrived;       // last interval's arrivals (1 = steady, >=2 = burst)
		double stutterPer1k;  // residual render-starves per 1000 presents (validation)
		double meanPressure;  // mean decaying loss pressure (threshold x-axis)
		double score;         // objective = stutterPer1k + 10*(avgTarget-1)
		double p1, p2, p3;    // current fitted thresholds
	};
	TuneView getTuneView();
	void waitForFrame(double timeoutMs);
	bool renderOnMainThread(std::shared_ptr<moonlight_xbox_dx::VideoRenderer> &sceneRenderer);
	bool waitBeforePresent(int64_t deadline);
	int64_t getCurrentFramePts();
	int64_t getNextVBlankQpc(int64_t *now);
	void submitFrame(AVFrame *frame);

  private:
	Pacer();
	Pacer(const Pacer &) = delete;
	Pacer &operator=(const Pacer &) = delete;

	inline bool stopping() const noexcept {
		return m_Stopping.load(std::memory_order_acquire);
	}

	inline bool running() const noexcept {
		return m_Running.load(std::memory_order_acquire);
	}

	bool renderModeImmediate(std::shared_ptr<moonlight_xbox_dx::VideoRenderer> &sceneRenderer);
	bool renderModeDisplayLocked(std::shared_ptr<moonlight_xbox_dx::VideoRenderer> &sceneRenderer);
	void vsyncHardware();
	void updateFrameStats();

	std::shared_ptr<DX::DeviceResources> m_DeviceResources;
	std::thread m_VsyncThread;
	std::atomic<bool> m_Running{false};
	std::atomic<bool> m_Stopping{false};
	int m_StreamFps;
	double m_RefreshRate;
	std::atomic<int> m_PacingMode{PACING_ADAPTIVE};
	// Rolling history of pre-dequeue queue depth, used by PACING_QT to drop only
	// on a persistent backlog (touched only on the render thread).
	std::deque<int> m_QueueDepthHistory;
	// Smoothed frame-arrival jitter (ms), measured on the decoder thread in
	// submitFrame and read by PACING_ADAPTIVE to size the buffer target.
	std::atomic<double> m_ArrivalJitterMs{0.0};
	int64_t m_LastEnqueueQpc = 0;
	bool m_HaveLastEnqueue = false;
	// PACING_ADAPTIVE buffer controller. Judder source = frames the decoder/network lost,
	// detected at ENQUEUE via a PTS discontinuity (a dropped frame jumps pts by ~2 periods)
	// -- a producer-side signal, independent of the render buffer, so it does NOT collapse
	// when the buffer engages (no oscillation, unlike a render-side starve). We grow the
	// target fast on a decaying count of recent losses and shrink it slowly. The controller
	// state below is render-thread only (no atomics) except the two cross-thread atomics.
	double m_StarvePressure = 0.0;   // decaying count of recent lost/late frames
	int    m_AdaptiveTarget = 1;     // committed drop target (grow fast, shrink slow)
	int    m_ShrinkHoldFrames = 0;   // presents the lower demand has held (shrink gate)
	uint64_t m_LastLostSeen = 0;     // lost-frame events consumed so far (render thread)
	std::atomic<int> m_AdaptiveTargetPublished{1};   // live target for the overlay/CSV
	std::atomic<uint64_t> m_LostFrameEvents{0};      // lost frames seen at enqueue (decoder->render)
	int64_t m_LastFramePts = 0;      // previous enqueued pts (decoder thread, PTS gap detect)
	bool    m_HaveLastPts = false;   // decoder thread only
	// Primary growth signal: arrivals-per-present-interval. submitFrame (decoder) counts every
	// delivered frame; the render thread reads+clears it once per present, so the value is how
	// many frames piled up between two on-screen presents. Steady state == 1; a decode
	// stall+catch-up, a jitter clump, or a consumer present-hitch all deliver >= 2 in one
	// interval -- and unlike a starve/drop, this count does NOT change when the buffer engages
	// (holding more frames doesn't alter how many ARRIVE between presents), so it can't
	// oscillate. m_RecentBurst is its decaying max -> the buffer depth needed to absorb it.
	std::atomic<int> m_ArrivalsSincePresent{0};  // decoder thread increments, render clears
	double m_RecentBurst = 1.0;                  // decaying max of arrivals/interval (render thread)
	int    m_LastArrived = 0;                    // last interval's arrivals (overlay only)
	// Live-tunable PACING_ADAPTIVE constants (defaults here; overridable on-device via
	// LocalState\pacing_params.txt + loadTuningParams, so no rebuild to tune).
	std::atomic<double> m_pStarveForget{0.997};          // per-present decay (~2.8 s memory @120fps)
	std::atomic<double> m_pStarveP1{2.0};                // pressure -> target 2
	std::atomic<double> m_pStarveP2{6.0};                // pressure -> target 3
	std::atomic<double> m_pStarveP3{12.0};               // pressure -> target 4
	std::atomic<int>    m_pStarveShrinkHoldFrames{120};  // presents of lower demand before stepping down
	std::atomic<double> m_pBurstForget{0.995};           // arrivals-burst decay (~1.8 s reclaim @120fps)

	// --- On-device auto-tuner (debug) -----------------------------------------------
	// Per-condition COUNT accumulators (not EWMA snapshots), bucketed by the auto-detected
	// scene class. recomputeWeights() fits p1/p2/p3 directly from the measured loss-burst
	// distribution -- a buffer of depth 1+burst absorbs a burst of that size, so the needed
	// depth is read straight off the producer-side signal, independent of the controller's
	// current target (non-circular: converges in one pass). Counts accumulate over the whole
	// measurement window so scene length doesn't bias the fit; cleared on init / Reset logs /
	// after Recompute (each Optimize starts a fresh measurement window).
	int  detectedCondition();          // current auto-classified scene (from Stats)
	int  neededDepthFor(int cond);     // depth this scene needs, from its burst distribution
	void clearTunerStats();            // zero every accumulator (fresh measurement window)
	// Loss-burst histogram: the decoder thread (submitFrame) increments by burst size 1..4,
	// the UI thread reads/zeroes it -> atomic. Index 0 is unused.
	std::atomic<uint64_t> m_CondBurst[COND_COUNT][5] = {};
	// Render-thread accumulators (UI thread reads/zeroes; an 8-byte aligned double R/W is
	// atomic on x64 and the reset race is benign for a debug tuner).
	double m_CondPresents[COND_COUNT]  = {0.0, 0.0, 0.0}; // adaptive ticks observed
	double m_CondStarves[COND_COUNT]   = {0.0, 0.0, 0.0}; // render-side starves (residual judder)
	double m_CondPressSum[COND_COUNT]  = {0.0, 0.0, 0.0}; // sum of loss pressure (-> mean)
	double m_CondTargetSum[COND_COUNT] = {0.0, 0.0, 0.0}; // sum of committed target (-> mean)

	FrameCadence m_FrameCadence;
	AVFrame* m_CurrentFrame = nullptr;

	static constexpr int VSYNC_HISTORY_SIZE = 512;
	std::mutex m_FrameStatsLock;
	UINT m_LastSyncRefreshCount;
	int64_t m_LastSyncQpc;
	int64_t m_VsyncIntervalQpc;
	std::array<int64_t, VSYNC_HISTORY_SIZE> m_vhistory{};
	int m_vhcount = 0;
	int m_vhidx = 0;
	int64_t m_vhsum = 0;
	std::atomic<int64_t> m_LastSyncTarget{0};
	double m_ewmaVsyncDriftQpc = 1;
};
