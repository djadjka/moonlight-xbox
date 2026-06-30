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
	std::atomic<int> m_PacingMode{PACING_DRAIN};
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
