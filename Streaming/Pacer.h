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

	// Frame pacing mode (selected in host settings / quick menu). IMMEDIATE renders the
	// newest decoded frame and drops older ones for lowest latency; ADAPTIVE additionally
	// buffers to the measured arrival jitter to absorb transient spikes; DISPLAY_LOCKED
	// presents every vblank via the cadence accumulator.
	enum PacingMode {
		PACING_IMMEDIATE = 0,
		PACING_DISPLAY_LOCKED = 1,
		PACING_ADAPTIVE = 2,
		PACING_MODE_COUNT = 3,
	};

	void deinit();
	void init(const std::shared_ptr<DX::DeviceResources> &res, int maxVideoFps, double refreshRate, int pacingMode);
	bool getPacingImmediate();  // = mode != DISPLAY_LOCKED (kept for stats compatibility)
	int getPacingMode();
	void setPacingMode(int mode);
	void waitForFrame(double timeoutMs);
	bool renderOnMainThread(std::shared_ptr<moonlight_xbox_dx::VideoRenderer> &sceneRenderer);
	bool waitBeforePresent(int64_t deadline);
	int64_t getCurrentFramePts();
	int64_t getNextVBlankQpc(int64_t *now);
	void submitFrame(AVFrame *frame);
	void observeDecodeMs(double decodeMs);  // decoder thread: per-frame decode time (PACING_ADAPTIVE)
	// Live PACING_ADAPTIVE signals/state for the stats overlay + CSV trace.
	int    getAdaptiveTarget();             // current buffer depth target
	double getRecentMaxDecodeMs();          // decaying-max decode time (complex-scene signal)
	double getArrivalJitterMs();            // RFC 3550 arrival jitter (network signal)
	int    getCurrentHwm();                 // current FrameQueue high-water
	double getPhaseMarginMinMs();           // decaying-min arrival-to-vblank margin (explanatory metric)
	double getArrivalBurstScore();          // decaying count of clustered arrivals (3rd controller signal)

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
	double vblankPhaseMarginMs(int64_t nowQpc);  // ms from nowQpc to the next FULL-vblank flip; -1 until vsync tracking is up

	std::shared_ptr<DX::DeviceResources> m_DeviceResources;
	std::thread m_VsyncThread;
	std::atomic<bool> m_Running{false};
	std::atomic<bool> m_Stopping{false};
	int m_StreamFps;
	double m_RefreshRate;
	std::atomic<int> m_PacingMode{PACING_IMMEDIATE};
	// Smoothed frame-arrival jitter (ms, RFC 3550), measured in submitFrame on the decoder
	// thread; read by PACING_ADAPTIVE to size the drop target.
	std::atomic<double> m_ArrivalJitterMs{0.0};
	int64_t m_LastEnqueueQpc = 0;
	bool m_HaveLastEnqueue = false;
	// Decaying max of recent per-frame decode time (ms), fed by observeDecodeMs on the decoder
	// thread (non-IDR frames only). A complex scene -> decode overruns the frame budget -> this
	// rises -> PACING_ADAPTIVE buffers; it decays back so the buffer reclaims latency when decode
	// gets easy. Buffer-independent (decode duration is upstream of the queue) -> can't oscillate.
	std::atomic<double> m_RecentMaxDecodeMs{0.0};
	// Decaying MIN of the arrival-to-next-vblank margin (ms), written in submitFrame on the
	// decoder thread, read by overlay/CSV; -1 until the first value. MEASUREMENT ONLY for now:
	// candidate 3rd controller signal (delivery-phase health). Buffer-independent by design —
	// arrivals are set by the network/host, vblank grid by the display; the queue affects neither.
	std::atomic<double> m_PhaseMarginMinMs{-1.0};
	// Decaying score of clustered arrivals (bursts), written in submitFrame on the decoder
	// thread. A starve+catch-up cycle at depth 1 always produces a producer-side burst (the
	// late frame arrives clustered with the next one), so this is the delivery-phase controller
	// signal: it persists regardless of buffer depth (arrivals don't change when we buffer),
	// which is what makes it oscillation-proof, unlike render-side starve/drop counts.
	std::atomic<double> m_ArrivalBurstScore{0.0};
	int m_LastHwm = 3;  // render thread: last high-water set (== FRAME_QUEUE_HIGH; avoids re-locking)
	// Render-thread controller state: burst-signal hysteresis latch (enter 1.5 / release 0.5)
	// and the lazily-shrunk effective target (grow instant, shrink only at drain moments AND
	// after the desired value stayed below it for the dwell, so reclaim never drops a frame
	// and momentary signal dips can't lower the ceiling into an oncoming clump).
	bool m_BurstLatched = false;
	int m_EffectiveTarget = 1;
	int64_t m_ShrinkArmedQpc = 0;  ///< when the desired target first dipped below the effective one
	std::atomic<int> m_AdaptiveTargetPublished{1};  // current target, for the overlay/CSV

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
