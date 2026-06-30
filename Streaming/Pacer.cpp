// clang-format off
#include "pch.h"
// clang-format on
#include "Pacer.h"
#include <algorithm>
#include <chrono>
#include <fstream>
#include <thread>
#include <windows.h>
#include "../Plot/ImGuiPlots.h"
#include "FFmpegDecoder.h"
#include "FrameQueue.h"
#include "Utils.hpp"

// Frame Pacing operation
//
// 3 threads use this class:
//
// Decoder thread (run from moonlight-common-c because DIRECT_SUBMIT)
//   * calls submitFrame() to queue a new AVFrame to FrameQueue class via FrameQueue::instance().enqueue(frame)
//   * Frames are dropped at enqueue time, in an alternating manner, when high water mark (default 2 + 1) is exceeded
//   * IDR frames are never dropped
//
// vsyncHardware thread:
//   * low-priority background thread responsible for tracking accurate vsync stats via GetFrameStatistics()
//
// main render loop thread:
//   * calls waitForFrame() with a timeout, to wait for new frames to become available in FrameQueue
//   * calls renderOnMainThread to render decoded video frame via VideoRenderer
//   * calls waitBeforePresent() using vsync timing data to align with the next vblank interval (or half-vblank for 120hz on Xbox)
//
// Calls to FQLog() and functions called within FQLog() are no-op unless you define FRAME_QUEUE_VERBOSE in pch.h
// and build in Debug mode.

constexpr int FRAME_QUEUE_LOW = 1;
constexpr int FRAME_QUEUE_HIGH = 3;

// On-device auto-tuner (debug) fit constants. The needed buffer depth for a scene is read
// off its loss-burst distribution; these gate noise and keep clean scenes at the floor.
constexpr double kTunerMinSamples   = 500.0; // presents a scene needs before it counts (~4 s @120)
constexpr double kTunerBurstFloor1k = 5.0;   // a burst size must recur >= this/1k to buffer for it
                                             // (rejects ~1 s of transition-lag misclassification;
                                             //  genuine drop scenes lose frames at 10s-100s/1k)
constexpr double kTunerStutter1k    = 5.0;   // residual starves/1k above which a depth is under-buffered
constexpr double kTunerP1Floor      = 1.0;   // never set p1 below this -> clean (pressure ~0) stays depth 1

// PACING_ADAPTIVE buffer controller. The judder signal is the producer-side lost-frame
// count (decoder/network drops, detected at enqueue via a PTS gap -- see submitFrame);
// m_StarvePressure is its decaying count. It maps to a buffer depth, grows fast and
// shrinks slowly so the buffer engages only while loss persists and returns to 1
// (== DRAIN latency) when the link is clean. The constants (forget/p1/p2/p3/shrink_hold)
// are live-tunable members, defaulted in Pacer.h and reloadable via loadTuningParams.

using steady_clock = std::chrono::steady_clock;
using namespace moonlight_xbox_dx;

Pacer &Pacer::instance() {
	static Pacer inst;
	return inst;
}

Pacer::Pacer()
    : m_Running(false),
      m_DeviceResources(),
      m_VsyncThread(),
      m_Stopping(false),
      m_StreamFps(0),
      m_RefreshRate(0.0),
      m_FrameCadence(),
      m_LastSyncRefreshCount(0),
      m_LastSyncQpc(0),
      m_VsyncIntervalQpc(0) {
}

void Pacer::deinit() {
	m_Running.store(false, std::memory_order_release);
	m_Stopping.store(true, std::memory_order_release);

	// Stop and clear out FrameQueue
	FrameQueue::instance().stop();

	// Stop the vsync thread
	if (m_VsyncThread.joinable()) {
		m_VsyncThread.join();
	}

	m_DeviceResources = nullptr;

	if (m_CurrentFrame) {
		av_frame_free(&m_CurrentFrame);
		m_CurrentFrame = nullptr;
	}

	Utils::Logf("Pacer: deinit\n");
}

void Pacer::init(const std::shared_ptr<DX::DeviceResources> &res, int streamFps, double refreshRate, bool framePacingImmediate) {
	m_Stopping.store(false, std::memory_order_release);
	m_DeviceResources = res;
	m_StreamFps = streamFps;
	m_RefreshRate = refreshRate;
	// Keep the configured default mode, but don't downgrade a user-selected
	// Immediate strategy (qt/adaptive) to plain drain on reinit.
	if (!framePacingImmediate) {
		m_PacingMode.store(PACING_DISPLAY_LOCKED, std::memory_order_release);
	} else if (m_PacingMode.load(std::memory_order_acquire) == PACING_DISPLAY_LOCKED) {
		m_PacingMode.store(PACING_ADAPTIVE, std::memory_order_release);
	}

	// Pick up any on-device tuning overrides for the adaptive controller (no rebuild).
	loadTuningParams();

	m_FrameCadence.init(m_RefreshRate > 0.0 ? m_RefreshRate : 60.0, static_cast<double>(streamFps));

	Utils::Logf("Frame Pacer init: mode %s, streamFps %d, refreshRate %.2f\n",
	            framePacingImmediate ? "immediate" : "display-locked", m_StreamFps, m_RefreshRate);

	m_vhsum = 0;
	m_vhcount = 0;
	m_vhidx = 0;
	std::fill(m_vhistory.begin(), m_vhistory.end(), 0);
	m_VsyncIntervalQpc = 0;
	m_LastSyncTarget = 0;
	m_ewmaVsyncDriftQpc = MsToQpc(0.0001);

	// Reset the PACING_ADAPTIVE controller so a reconnect starts at the low-latency floor.
	m_StarvePressure = 0.0;
	m_AdaptiveTarget = 1;
	m_ShrinkHoldFrames = 0;
	m_LastLostSeen = 0;
	m_AdaptiveTargetPublished.store(1, std::memory_order_release);
	m_LostFrameEvents.store(0, std::memory_order_release);
	m_HaveLastPts = false;
	m_ArrivalsSincePresent.store(0, std::memory_order_release);
	m_RecentBurst = 1.0;
	m_LastArrived = 0;
	clearTunerStats();

	// Start FrameQueue so it's ready to receive new frames
	FrameQueue::instance().setHighWaterMark(FRAME_QUEUE_HIGH);
	FrameQueue::instance().start();

	if (!m_VsyncThread.joinable()) {
		m_VsyncThread = std::thread(&Pacer::vsyncHardware, this);
	}

	m_Running.store(true, std::memory_order_release);
}

bool Pacer::getPacingImmediate() {
	return m_PacingMode.load(std::memory_order_acquire) != PACING_DISPLAY_LOCKED;
}

void Pacer::setPacingImmediate(bool framePacingImmediate) {
	// Quick flip between the two primary modes (used by the #253 menu item).
	m_PacingMode.store(framePacingImmediate ? PACING_DRAIN : PACING_DISPLAY_LOCKED,
	                   std::memory_order_release);
}

int Pacer::getPacingMode() {
	return m_PacingMode.load(std::memory_order_acquire);
}

void Pacer::setPacingMode(int mode) {
	if (mode < 0 || mode >= PACING_MODE_COUNT) {
		mode = PACING_DRAIN;
	}
	m_PacingMode.store(mode, std::memory_order_release);
}

int Pacer::getAdaptiveTarget() {
	return m_AdaptiveTargetPublished.load(std::memory_order_acquire);
}

// The decaying producer-side loss count the adaptive controller reacts to. Stats reads it to
// classify the scene from the SAME signal the tuner uses, so the label can't disagree with
// the buffer's behaviour (~0 in a clean scene, elevated under sustained loss). Updated on the
// render thread; Stats::logCsvLine reads it on that same thread, so the read is race-free.
double Pacer::getRecentLossPressure() {
	return m_StarvePressure;
}

// Re-read the PACING_ADAPTIVE controller constants from LocalState\pacing_params.txt so
// they can be tuned on-device without a rebuild. Format: one "key=value" per line, keys
// forget / p1 / p2 / p3 / shrink_hold. Missing file or keys keep the current values.
void Pacer::loadTuningParams() {
	try {
		auto folder = Windows::Storage::ApplicationData::Current->LocalFolder;
		std::wstring path(folder->Path->Data());
		path += L"\\pacing_params.txt";
		std::ifstream f(path.c_str());
		if (!f.is_open()) {
			Utils::Logf("Pacer: pacing_params.txt not found, keeping current adaptive tuning\n");
			return;
		}
		auto trim = [](std::string &s) {
			size_t a = s.find_first_not_of(" \t\r\n");
			size_t b = s.find_last_not_of(" \t\r\n");
			if (a == std::string::npos) { s.clear(); return; }
			s = s.substr(a, b - a + 1);
		};
		std::string line;
		while (std::getline(f, line)) {
			size_t eq = line.find('=');
			if (eq == std::string::npos) continue;
			std::string key = line.substr(0, eq), val = line.substr(eq + 1);
			trim(key); trim(val);
			if (key.empty() || val.empty() || key[0] == '#') continue;
			try {
				if (key == "forget")           m_pStarveForget.store(std::stod(val), std::memory_order_relaxed);
				else if (key == "p1")          m_pStarveP1.store(std::stod(val), std::memory_order_relaxed);
				else if (key == "p2")          m_pStarveP2.store(std::stod(val), std::memory_order_relaxed);
				else if (key == "p3")          m_pStarveP3.store(std::stod(val), std::memory_order_relaxed);
				else if (key == "shrink_hold") m_pStarveShrinkHoldFrames.store(std::stoi(val), std::memory_order_relaxed);
				else if (key == "burst_forget") m_pBurstForget.store(std::stod(val), std::memory_order_relaxed);
			} catch (...) { /* skip a malformed value */ }
		}
		Utils::Logf("Pacer adaptive tuning: forget=%.4f p1=%.2f p2=%.2f p3=%.2f shrink_hold=%d burst_forget=%.4f\n",
		            m_pStarveForget.load(), m_pStarveP1.load(), m_pStarveP2.load(),
		            m_pStarveP3.load(), m_pStarveShrinkHoldFrames.load(), m_pBurstForget.load());
	} catch (...) {
		// best effort
	}
}

// Clear the CSV pacing trace AND the auto-tuner stats so the next on-device measurement
// starts from a clean slate (debug button).
void Pacer::resetTraceLogs() {
	if (m_DeviceResources && m_DeviceResources->GetStats()) {
		m_DeviceResources->GetStats()->resetCsv();
	}
	clearTunerStats();
}

// Zero every auto-tuner accumulator so the next measurement window starts clean. Called on
// init, by the Reset-logs button, and at the end of recomputeWeights (fresh iteration).
void Pacer::clearTunerStats() {
	for (int c = 0; c < COND_COUNT; ++c) {
		m_CondPresents[c] = 0.0;
		m_CondStarves[c] = 0.0;
		m_CondPressSum[c] = 0.0;
		m_CondTargetSum[c] = 0.0;
		for (int k = 0; k < 5; ++k) {
			m_CondBurst[c][k].store(0, std::memory_order_relaxed);
		}
	}
}

// Depth this scene needs, read straight off its loss-burst distribution: a buffer of depth
// 1+burst keeps the on-screen cadence smooth across a gap of that size. Bursts that don't
// recur often enough (a one-off seek/reinit) are ignored; a residual starve at the
// burst-sized depth (late arrivals that weren't flagged as a PTS gap) bumps it one more.
int Pacer::neededDepthFor(int c) {
	if (c < 0 || c >= COND_COUNT) return 1;
	double n = m_CondPresents[c];
	if (n < kTunerMinSamples) return 1; // too little data: assume clean (low-latency floor)
	int maxK = 0;
	for (int k = 1; k <= 4; ++k) {
		double rate1k = static_cast<double>(m_CondBurst[c][k].load(std::memory_order_relaxed)) / n * 1000.0;
		if (rate1k >= kTunerBurstFloor1k) {
			maxK = k; // bursts of this size recur often enough to permanently buffer for
		}
	}
	int need = 1 + maxK;
	double stut1k = m_CondStarves[c] / n * 1000.0;
	if (stut1k > kTunerStutter1k && need < 4) {
		need += 1; // still starving at the burst-sized depth -> one deeper
	}
	if (need > 4) need = 4;
	return need;
}

// Current scene class, auto-detected by Stats from the per-second loss breakdown. Render
// thread reads it each present to bucket the auto-tuner stats -- no manual labelling.
int Pacer::detectedCondition() {
	if (m_DeviceResources && m_DeviceResources->GetStats()) {
		int c = m_DeviceResources->GetStats()->getDetectedCondition();
		if (c >= 0 && c < COND_COUNT) {
			return c;
		}
	}
	return COND_CLEAN;
}

Pacer::TuneView Pacer::getTuneView() {
	TuneView v{};
	int c = detectedCondition();
	v.cond = c;
	double n = m_CondPresents[c];
	v.samples = n;
	double loss1k = 0.0;
	int maxK = 0;
	for (int k = 1; k <= 4; ++k) {
		uint64_t cnt = m_CondBurst[c][k].load(std::memory_order_relaxed);
		if (n > 0.0) loss1k += static_cast<double>(cnt) / n * 1000.0;
		if (cnt > 0) maxK = k;
	}
	v.lossPer1k = loss1k;
	v.maxBurst = maxK;
	v.neededDepth = neededDepthFor(c);
	v.avgTarget = n > 0.0 ? m_CondTargetSum[c] / n : 1.0;
	v.recentBurst = m_RecentBurst;
	v.arrived = m_LastArrived;
	v.stutterPer1k = n > 0.0 ? m_CondStarves[c] / n * 1000.0 : 0.0;
	v.meanPressure = n > 0.0 ? m_CondPressSum[c] / n : 0.0;
	v.score = v.stutterPer1k + 10.0 * (v.avgTarget - 1.0);
	v.p1 = m_pStarveP1.load(std::memory_order_relaxed);
	v.p2 = m_pStarveP2.load(std::memory_order_relaxed);
	v.p3 = m_pStarveP3.load(std::memory_order_relaxed);
	return v;
}

// Fit p1/p2/p3 from the per-condition stats and apply them live + persist. Each scene's
// needed depth is read off its loss-burst distribution (neededDepthFor: depth 1+burst
// absorbs a burst of that size) -- a producer-side measurement, independent of the depth
// the controller is currently choosing, so the fit is NOT circular and converges in one
// pass. Each higher-depth threshold is then placed just below the loss pressure of the
// lightest scene that still needs that depth, so the controller steps up exactly when such
// a scene is active and no sooner.
void Pacer::recomputeWeights() {
	int    needed[COND_COUNT];
	double meanPress[COND_COUNT];
	for (int c = 0; c < COND_COUNT; ++c) {
		needed[c] = neededDepthFor(c);
		meanPress[c] = m_CondPresents[c] > 0.0 ? m_CondPressSum[c] / m_CondPresents[c] : 0.0;
	}

	double newP[3] = {1e9, 1e9, 1e9}; // p1,p2,p3; 1e9 = that depth is never reached
	for (int level = 2; level <= 4; ++level) {
		double minP = 1e9;
		for (int c = 0; c < COND_COUNT; ++c) {
			// Only a scene with enough data AND real loss pressure can place a pressure
			// threshold. This rejects clean/jitter scenes (pressure ~0, but a render starve
			// from a late-but-present frame can bump needed depth) from dragging p1 to the
			// floor -- a zero-pressure scene can't meaningfully sit on the pressure axis.
			if (needed[c] >= level && m_CondPresents[c] >= kTunerMinSamples &&
			    meanPress[c] >= kTunerP1Floor && meanPress[c] < minP) {
				minP = meanPress[c];
			}
		}
		if (minP < 1e9) {
			newP[level - 2] = minP * 0.9; // sit just below the observed pressure
		}
	}
	if (newP[0] < kTunerP1Floor) newP[0] = kTunerP1Floor; // keep clean (pressure ~0) at depth 1
	if (newP[1] < newP[0]) newP[1] = newP[0];             // keep thresholds monotonic
	if (newP[2] < newP[1]) newP[2] = newP[1];
	m_pStarveP1.store(newP[0], std::memory_order_relaxed);
	m_pStarveP2.store(newP[1], std::memory_order_relaxed);
	m_pStarveP3.store(newP[2], std::memory_order_relaxed);

	// Persist so the fit survives a reconnect and is visible/editable in pacing_params.txt.
	try {
		auto folder = Windows::Storage::ApplicationData::Current->LocalFolder;
		std::wstring path(folder->Path->Data());
		path += L"\\pacing_params.txt";
		std::ofstream f(path.c_str(), std::ios::trunc);
		if (f.is_open()) {
			f << "# recomputed on-device\n";
			f << "forget=" << m_pStarveForget.load() << "\n";
			f << "p1=" << newP[0] << "\n";
			f << "p2=" << newP[1] << "\n";
			f << "p3=" << newP[2] << "\n";
			f << "shrink_hold=" << m_pStarveShrinkHoldFrames.load() << "\n";
			f << "burst_forget=" << m_pBurstForget.load() << "\n";
		}
	} catch (...) {
		// best effort
	}
	Utils::Logf("Pacer recompute: need[clean=%d pacing=%d net=%d] press[%.2f/%.2f/%.2f] -> p1=%.2f p2=%.2f p3=%.2f\n",
	            needed[COND_CLEAN], needed[COND_PACING], needed[COND_NETWORK],
	            meanPress[COND_CLEAN], meanPress[COND_PACING], meanPress[COND_NETWORK],
	            newP[0], newP[1], newP[2]);

	// Start the next iteration from a clean slate so it measures only behaviour under the
	// weights just applied (no carry-over from prior iterations).
	clearTunerStats();
}

void Pacer::vsyncHardware() {
	if (!SetThreadPriority(GetCurrentThread(), THREAD_PRIORITY_ABOVE_NORMAL)) {
		Utils::Logf("Failed to set vsyncHardware priority: %d\n", GetLastError());
	}

	Utils::Logf("vsyncHardware stats thread started, qpcFreq=%lld ticksPerMs=%lld\n",
	            QpcFreq(), MsToQpc(1.0));

	while (!stopping()) {
		// All this thread does is wake up every vsync and record the precise vsync QPC the system
		// tracks. This data is several frames out of date but it's enough to
		// very precisely time present calls and to determine the vsync interval.
		m_DeviceResources->GetDXGIOutput()->WaitForVBlank();
		updateFrameStats();
	}

	Utils::Logf("vsyncHardware stats thread stopped\n");
}

// based on mpv's d3d11_get_vsync()
void Pacer::updateFrameStats() {
	std::scoped_lock<std::mutex> lock(m_FrameStatsLock);

	// After we've presented a couple of frames, we can obtain the true vsync interval
	DXGI_FRAME_STATISTICS stats;
	if (m_DeviceResources->GetSwapChain()->GetFrameStatistics(&stats) == S_OK &&
	    (stats.SyncRefreshCount != 0 || stats.SyncQPCTime.QuadPart != 0ULL)) {
		UINT srcPassed = 0;
		if (stats.SyncRefreshCount && m_LastSyncRefreshCount) {
			srcPassed = stats.SyncRefreshCount - m_LastSyncRefreshCount;
		}
		m_LastSyncRefreshCount = stats.SyncRefreshCount;

		int64_t sqtPassed = 0;
		if (stats.SyncQPCTime.QuadPart && m_LastSyncQpc) {
			sqtPassed = stats.SyncQPCTime.QuadPart - m_LastSyncQpc;
		}
		m_LastSyncQpc = stats.SyncQPCTime.QuadPart;

		// compare with the last sync target we used in waitBeforePresent
		int64_t driftQpc = 0;
		int64_t lastTargetQpc = m_LastSyncTarget.load(std::memory_order_acquire);
		if (lastTargetQpc) {
			driftQpc = m_LastSyncQpc - lastTargetQpc;
			if (std::llabs(driftQpc) < MsToQpc(0.03)) {
				const double alpha = 0.05; // slow moving average
				m_ewmaVsyncDriftQpc = (1.0 - alpha) * m_ewmaVsyncDriftQpc + alpha * static_cast<double>(driftQpc);
			}
		}

		// If any vsyncs have passed, we can calculate a very accurate interval
		if (srcPassed && sqtPassed) {
			const int64_t intervalQpc = sqtPassed / srcPassed;

			// use average from past 10 intervals
			if (m_vhcount == VSYNC_HISTORY_SIZE) {
				m_vhsum -= m_vhistory[m_vhidx];
			} else {
				++m_vhcount;
			}
			m_vhsum += intervalQpc;
			m_vhistory[m_vhidx] = intervalQpc;
			m_vhidx = (m_vhidx + 1) % VSYNC_HISTORY_SIZE;

			// Compute average
			m_VsyncIntervalQpc = m_vhsum / m_vhcount;

			FQLog("updateFrameStats(): LastSyncQpc %lld, Estimated vsync interval: %.3fms (%.2f Hz) (%lld ticks), "
			      "driftQpc %lld (%fms), driftAvg %lld\n",
			      m_LastSyncQpc, QpcToMs(m_VsyncIntervalQpc), 1000.0 / QpcToMs(m_VsyncIntervalQpc), m_VsyncIntervalQpc,
			      driftQpc, QpcToMs(driftQpc), static_cast<int64_t>(m_ewmaVsyncDriftQpc));
		}
	} else {
		// We have a chicken and the egg problem here in that no frame stats are available before presenting real frames,
		// so we need to fake some numbers early on so Pacer can at least limp through a few frames.
		double vsyncRR = m_RefreshRate;

		if (IsXbox()) {
			if (vsyncRR >= 120.0) {
				vsyncRR = 60.0;
			} else if (vsyncRR >= 119.0) {
				vsyncRR = 59.94;
			} else if (vsyncRR >= 60.0) {
				vsyncRR = 60.0;
			} else if (vsyncRR >= 59.0) {
				vsyncRR = 59.94;
			}
		}

		m_LastSyncQpc = QpcNow();
		m_VsyncIntervalQpc = MsToQpc(1000.0 / vsyncRR);

		LogOnce("vsyncHardware(): starting up with interval %.2f based on system rate %.2f\n", vsyncRR, m_RefreshRate);
	}
}

// Main render thread

void Pacer::waitForFrame(double timeoutMs) {
	if (!running()) return;

	// Wait for a decoded frame to be available
	const int queueHas = 1;
	FrameQueue::instance().waitForEnqueue(queueHas, timeoutMs);
}

// called by render thread
bool Pacer::renderOnMainThread(std::shared_ptr<VideoRenderer> &sceneRenderer) {
	if (!running()) return false;

	if (m_PacingMode.load(std::memory_order_acquire) == PACING_DISPLAY_LOCKED) {
		return renderModeDisplayLocked(sceneRenderer);
	} else {
		return renderModeImmediate(sceneRenderer);
	}
}

// Dequeue a new frame if available and immediately render it. When no new frame is available
// skips Present and relies on the system to continue showing the previous frame.
// Pros: lowest latency, output framerate matches input framerate
// Cons: only works well on Xbox Series for some reason
bool Pacer::renderModeImmediate(std::shared_ptr<VideoRenderer> &sceneRenderer) {
	const int mode = m_PacingMode.load(std::memory_order_acquire);
	int droppedToCatchUp = 0;
	AVFrame *newFrame = nullptr;

	if (mode == PACING_DRAIN) {
		// Lowest latency: render the NEWEST available frame and discard any older ones,
		// so no standing buffer can accumulate. In steady state (<= 1 queued) the loop
		// finds nothing extra and drops nothing.
		newFrame = FrameQueue::instance().dequeue();
		if (!newFrame) {
			return false; // no frame, don't Present()
		}
		for (AVFrame *newer; (newer = FrameQueue::instance().dequeue()) != nullptr;) {
			av_frame_free(&newFrame);
			newFrame = newer;
			++droppedToCatchUp;
		}
	} else if (mode == PACING_LEGACY) {
		// Original upstream off-by-one (single-shot catch-up). Kept selectable only for
		// on-device A/B against the metastable standing buffer it produces.
		newFrame = FrameQueue::instance().dequeue();
		if (!newFrame) {
			return false; // no frame, don't Present()
		}
		int queueDepth = FrameQueue::instance().count();
		if (queueDepth > FRAME_QUEUE_LOW) {
			AVFrame *newFrame2 = FrameQueue::instance().dequeue();
			if (newFrame2) {
				av_frame_free(&newFrame);
				newFrame = newFrame2;
				++droppedToCatchUp;
			}
		}
	} else {
		// PACING_QT / PACING_ADAPTIVE: buffer-target strategies. Drop the OLDEST frames
		// down to `target`, then render the oldest remaining (FIFO when buffering, newest
		// when target == 1). They differ only in how `target` is chosen.
		const int depth = FrameQueue::instance().count(); // sample before dropping
		int target = 1;                                   // strict default: drop to newest
		// Leniency only helps when the source can outpace the display; otherwise a backlog
		// can't be sustained by rate, so stay strict (lowest latency).
		if (m_StreamFps >= m_RefreshRate) {
			if (mode == PACING_QT) {
				// moonlight-qt hysteresis over a ~500 ms window of queue depth: lenient
				// (buffer up to 3) if the queue recently resolved to <= 1, else strict.
				for (int entry : m_QueueDepthHistory) {
					if (entry <= 1) {
						target = 3;
						break;
					}
				}
				int window = static_cast<int>(m_RefreshRate / 2.0);
				if (window < 1) {
					window = 1;
				}
				if (static_cast<int>(m_QueueDepthHistory.size()) >= window) {
					m_QueueDepthHistory.pop_front();
				}
				m_QueueDepthHistory.push_back(depth);
			} else {
				// PACING_ADAPTIVE: size the buffer to recent judder via a decaying controller.
				// Grow fast, shrink slow, so it engages only while judder persists and returns
				// to 1 (== DRAIN latency) when clean. Two signals feed it: arrivals-per-present
				// (primary, catches every contiguous-PTS pile-up) and producer-side PTS-gap loss
				// (secondary, catches genuine skips) -- both decoupled from the buffer depth so
				// the controller can't oscillate.
				uint64_t lostNow = m_LostFrameEvents.load(std::memory_order_acquire);
				double newLost = static_cast<double>(lostNow - m_LastLostSeen);
				m_LastLostSeen = lostNow;

				// PRIMARY signal: arrivals-per-present-interval. Clear the decoder-side counter
				// for this present; the value is how many frames piled up since the last one.
				// A burst of K (decode catch-up / jitter clump / consumer hitch) needs a buffer
				// ~K deep to present them one-per-vblank instead of dropping. Decaying max so it
				// grows on a burst and reclaims latency (-> 1) when arrivals settle back to 1.
				// Unlike a drop/starve, this count is independent of the buffer depth -> no
				// oscillation. This catches the contiguous-PTS drops the loss signal cannot see.
				int arrived = m_ArrivalsSincePresent.exchange(0, std::memory_order_relaxed);
				if (arrived > 8) arrived = 8; // clamp a mode-switch backlog
				m_LastArrived = arrived;
				m_RecentBurst *= m_pBurstForget.load(std::memory_order_relaxed);
				if ((double) arrived > m_RecentBurst) m_RecentBurst = (double) arrived;
				if (m_RecentBurst < 1.0) m_RecentBurst = 1.0;
				int desired = static_cast<int>(m_RecentBurst + 0.5);

				// Secondary: genuine producer-side loss (PTS gaps) -- decoder/network SKIPS,
				// which the arrivals signal doesn't see (a skipped frame never arrives).
				m_StarvePressure = m_StarvePressure * m_pStarveForget.load(std::memory_order_relaxed) + newLost;
				if (m_StarvePressure >= m_pStarveP1.load(std::memory_order_relaxed) && desired < 2) desired = 2;
				if (m_StarvePressure >= m_pStarveP2.load(std::memory_order_relaxed) && desired < 3) desired = 3;
				if (m_StarvePressure >= m_pStarveP3.load(std::memory_order_relaxed) && desired < 4) desired = 4;

				// Predictive floor: pre-buffer sustained network arrival jitter (RFC 3550
				// tail) before it turns into starves.
				const double frameMs = m_FrameCadence.streamPeriodMs();
				if (frameMs > 0.0) {
					const double jitterMs = m_ArrivalJitterMs.load(std::memory_order_acquire);
					int floorTarget = 1 + static_cast<int>(((jitterMs * 2.0) / frameMs) + 0.5);
					if (floorTarget > desired) {
						desired = floorTarget;
					}
				}
				if (desired > 4) { // cap (FrameQueue capacity is 5)
					desired = 4;
				}

				// Grow fast, shrink slow: jump up immediately; step down one only after the
				// lower demand has held for m_pStarveShrinkHoldFrames (anti-oscillation).
				const int prevTarget = m_AdaptiveTarget;
				if (desired > m_AdaptiveTarget) {
					m_AdaptiveTarget = desired;
					m_ShrinkHoldFrames = 0;
				} else if (desired < m_AdaptiveTarget) {
					if (++m_ShrinkHoldFrames >= m_pStarveShrinkHoldFrames.load(std::memory_order_relaxed)) {
						--m_AdaptiveTarget;
						m_ShrinkHoldFrames = 0;
					}
				} else {
					m_ShrinkHoldFrames = 0;
				}
				target = m_AdaptiveTarget;
				m_AdaptiveTargetPublished.store(target, std::memory_order_release);

				// Coordinate the enqueue cap with the target: the queue must be allowed to hold
				// `target` frames plus one arriving, or a grown buffer can't actually fill (the
				// high-water alternate-drop would discard the burst at enqueue before the render
				// loop could retain it). Only touch it on a change (it locks FrameQueue).
				if (m_AdaptiveTarget != prevTarget) {
					int hwm = m_AdaptiveTarget + 1;
					if (hwm < FRAME_QUEUE_HIGH) hwm = FRAME_QUEUE_HIGH;
					if (hwm > 5) hwm = 5; // FrameQueue capacity
					FrameQueue::instance().setHighWaterMark(hwm);
				}

				// Auto-tuner accumulator (debug): count this adaptive tick under the detected
				// scene and sum the signals the fit needs. Counts (not EWMAs) so the fit is
				// robust to scene length; the loss-burst histogram is filled at enqueue.
				const int cond = detectedCondition();
				m_CondPresents[cond]  += 1.0;
				m_CondPressSum[cond]  += m_StarvePressure;
				m_CondTargetSum[cond] += m_AdaptiveTarget;
			}
		}
		// Drop the OLDEST frames until we are at the target depth.
		while (FrameQueue::instance().count() > target) {
			AVFrame *old = FrameQueue::instance().dequeue();
			if (!old) {
				break;
			}
			av_frame_free(&old);
			++droppedToCatchUp;
		}
		newFrame = FrameQueue::instance().dequeue();
		if (!newFrame) {
			// Auto-tuner: a starve (repeated frame) is the residual judder at the current
			// target. Measurement only -- it does NOT feed the controller (that would
			// re-introduce oscillation); it just records how well this depth is working.
			if (mode == PACING_ADAPTIVE) {
				m_CondStarves[detectedCondition()] += 1.0;
			}
			return false; // no frame, don't Present()
		}
	}

	if (droppedToCatchUp > 0) {
		ImGuiPlots::instance().observeFloat(PLOT_DROPPED_PACER, (float) droppedToCatchUp);
	}

	if (m_CurrentFrame) {
		av_frame_free(&m_CurrentFrame);
	}
	m_CurrentFrame = newFrame;

	int64_t beforeRenderQpc = QpcNow();

	// Render it
	FQLog("> Frame rendered [pts: %.3f] [%.2ffps] [%.2fhz] [queued %d]\n",
	      m_CurrentFrame->pts / 90.0, m_FrameCadence.streamFps(), m_FrameCadence.displayHz(), FrameQueue::instance().count());

	if (!sceneRenderer->Render(m_CurrentFrame)) {
		return false; // something went wrong rendering the frame
	}

	if (m_CurrentFrame->opaque_ref) {
		// Count time spent in FrameQueue
		auto *data = reinterpret_cast<MLFrameData *>(m_CurrentFrame->opaque_ref->data);
		m_DeviceResources->GetStats()->SubmitPacerTime(beforeRenderQpc - data->decodeEndQpc);
	}

	// Keep m_CurrentFrame alive until next frame, it's used to calculate frametime
	return true; // ok to Present()
}

// Attempt to pace rendering based on observed framerate from host pts data
// Pros: always presents at max refresh rate, using either a new frame or a cached previous frame
//       Prevents most artifacts/tearing on Xbox One.
//       May do a better job with e.g. 24fps needing 3:2 pulldown
// Cons: higher latency
//       more difficult to control queue size, requires additional frame drop logic
bool Pacer::renderModeDisplayLocked(std::shared_ptr<VideoRenderer> &sceneRenderer) {
	// Consume frame(s) according to cadence
	int advanceCount = m_FrameCadence.decideAdvanceCount();

	// if the queue has too many frames in it, break the cadence and render or drop one extra
	int queueDepth = FrameQueue::instance().count();
	if (queueDepth > FRAME_QUEUE_LOW) {
		advanceCount++;
	}

	for (int i = 0; i < advanceCount; ++i) {
		AVFrame *newFrame = FrameQueue::instance().dequeue();
		if (!newFrame) {
			break;
		}

		if (m_CurrentFrame) {
			if (i > 0) {
				// advanceCount was > 1, so this is a dropped frame
				ImGuiPlots::instance().observeFloat(PLOT_DROPPED_PACER, 1.0);
			}
			av_frame_free(&m_CurrentFrame);
		}
		m_CurrentFrame = newFrame;
	}

	if (!m_CurrentFrame) {
		// No frame available yet
		return false;
	}

	int64_t beforeRenderQpc = QpcNow();

	// Render it
	FQLog("> Frame rendered [pts: %.3f] [%.2ffps] [%.2fhz] [advanceCount %d] [queued %d]\n",
	      m_CurrentFrame->pts / 90.0, m_FrameCadence.streamFps(), m_FrameCadence.displayHz(),
	      advanceCount, queueDepth);

	if (!sceneRenderer->Render(m_CurrentFrame)) {
		return false; // something went wrong rendering the frame
	}

	if (m_CurrentFrame->opaque_ref) {
		// Count time spent in FrameQueue
		auto *data = reinterpret_cast<MLFrameData *>(m_CurrentFrame->opaque_ref->data);
		m_DeviceResources->GetStats()->SubmitPacerTime(beforeRenderQpc - data->decodeEndQpc);
	}

	// Keep m_CurrentFrame alive in case we need to reuse it on the next present
	return true; // ok to Present()
}

// called by render thread, returns true if we waited, false if we missed the target
bool Pacer::waitBeforePresent(int64_t target) {
	if (!running()) return false;

	int64_t now = QpcNow();
	if (target <= 0) {
		target = getNextVBlankQpc(&now);
	}

	m_LastSyncTarget.store(target, std::memory_order_release);

	if (target > now) {
		FQLog("waitBeforePresent(): waiting %.3fms\n", QpcToMs(target - now));
		SleepUntilQpc(target);
		return true;
	}

	return false;
}

// called by render thread
int64_t Pacer::getCurrentFramePts() {
	if (m_CurrentFrame) {
		return m_CurrentFrame->pts;
	}
	return 0;
}

// end main thread

// called by decoder thread
void Pacer::submitFrame(AVFrame *frame) {
	// Update cadence from pts if available
	if (frame->pts) {
		m_FrameCadence.observeFramePts(frame->pts);
	}

	// Measure frame-arrival jitter as the RFC 3550 smoothed deviation of the actual
	// inter-arrival time from the expected frame period: J += (|D| - J)/16. Read by
	// PACING_ADAPTIVE to size the buffer target. Cheap (decoder-thread only).
	int64_t now = QpcNow();
	if (m_HaveLastEnqueue) {
		double arrivalDeltaMs = QpcToMs(now - m_LastEnqueueQpc);
		double dev = arrivalDeltaMs - m_FrameCadence.streamPeriodMs();
		if (dev < 0.0) {
			dev = -dev;
		}
		double j = m_ArrivalJitterMs.load(std::memory_order_acquire);
		m_ArrivalJitterMs.store(j + (dev - j) / 16.0, std::memory_order_release);
	}
	m_LastEnqueueQpc = now;
	m_HaveLastEnqueue = true;

	// Producer-side judder signal for PACING_ADAPTIVE: count frames the decoder/network
	// lost via a PTS discontinuity (a dropped frame jumps pts by ~2 periods). Measured
	// here at enqueue, so it stays elevated under sustained loss even after the buffer
	// hides the resulting render-side starves -> the controller can't oscillate.
	if (frame->pts && m_HaveLastPts) {
		double periodPts = m_FrameCadence.streamPeriodMs() * 90.0; // pts is 90 kHz
		if (periodPts > 0.0) {
			double deltaPts = static_cast<double>(frame->pts - m_LastFramePts);
			int lost = static_cast<int>((deltaPts / periodPts) + 0.5) - 1; // missing frames in the gap
			if (lost > 0) {
				if (lost > 4) {
					lost = 4; // clamp a discontinuity (seek/reinit) so it can't pin the buffer
				}
				m_LostFrameEvents.fetch_add(static_cast<uint64_t>(lost), std::memory_order_release);
				// Auto-tuner (debug): bucket the burst size under the current scene so
				// recomputeWeights can read the needed depth straight off the distribution.
				int c = detectedCondition();
				if (c >= 0 && c < COND_COUNT) {
					m_CondBurst[c][lost].fetch_add(1, std::memory_order_relaxed);
				}
			}
		}
	}
	if (frame->pts) {
		m_LastFramePts = frame->pts;
		m_HaveLastPts = true;
	}

	// Primary growth signal for PACING_ADAPTIVE: count this delivered frame. The render thread
	// clears this once per present, so it reads as arrivals-per-present-interval. Counted before
	// the enqueue drop so a burst that overflows the queue still registers its true size.
	m_ArrivalsSincePresent.fetch_add(1, std::memory_order_relaxed);

	int dropCount = FrameQueue::instance().enqueue(frame);
	if (dropCount) {
		m_DeviceResources->GetStats()->SubmitDroppedFrame(1);
	}

	ImGuiPlots::instance().observeFloat(PLOT_DROPPED_PACER, (float)dropCount);
	float avgQueueSize = ImGuiPlots::instance().observeFloatReturnAvg(PLOT_QUEUED_FRAMES, (float)FrameQueue::instance().count());
	m_DeviceResources->GetStats()->SubmitAvgQueueSize(avgQueueSize);
}

// Misc helper functions

// Caller often needs now and the vsync interval, since this needs locking
// the logic is confined to this function.
int64_t Pacer::getNextVBlankQpc(int64_t *now) {
	std::scoped_lock<std::mutex> lock(m_FrameStatsLock);
	int64_t target = 0, interval = 0;
	*now = QpcNow();

	if (m_LastSyncQpc == 0 || m_VsyncIntervalQpc == 0) {
		// Fallback until vsyncHardware spins up
		double rr = m_RefreshRate > 0.0 ? m_RefreshRate : 60.0;
		interval = MsToQpc(1000.0 / rr);
		target = *now + interval;
	} else {
		interval = m_VsyncIntervalQpc;
		int64_t next = m_LastSyncQpc + static_cast<int64_t>(m_ewmaVsyncDriftQpc);

		while (next < *now) {
			next += interval;
		}
		target = next;
	}

	if (IsXbox() && m_StreamFps == 120 && m_RefreshRate > 119.0) {
		// 120hz on Xbox requires us to present each frame at half-vsync intervals
		m_FrameCadence.setDisplayHz(1000.0 / QpcToMs(interval / 2));

		int64_t half = interval / 2;
		if (target - half > *now) {
			// we're currently in the first half of a vblank, sleep till the halfway mark
			target -= half;
		}
	} else {
		// Keep true refresh rate synced with cadence
		m_FrameCadence.setDisplayHz(1000.0 / QpcToMs(interval));
	}

	assert(target > *now);

	return target;
}
