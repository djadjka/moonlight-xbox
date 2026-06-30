// clang-format off
#include "pch.h"
// clang-format on
#include "Pacer.h"
#include <algorithm>
#include <chrono>
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

void Pacer::init(const std::shared_ptr<DX::DeviceResources> &res, int streamFps, double refreshRate, int pacingMode) {
	m_Stopping.store(false, std::memory_order_release);
	m_DeviceResources = res;
	m_StreamFps = streamFps;
	m_RefreshRate = refreshRate;
	if (pacingMode < 0 || pacingMode >= PACING_MODE_COUNT) {
		pacingMode = PACING_IMMEDIATE;
	}
	m_PacingMode.store(pacingMode, std::memory_order_release);

	m_FrameCadence.init(m_RefreshRate > 0.0 ? m_RefreshRate : 60.0, static_cast<double>(streamFps));

	const char *modeName = pacingMode == PACING_DISPLAY_LOCKED ? "display-locked"
	                       : pacingMode == PACING_ADAPTIVE     ? "adaptive"
	                                                           : "immediate";
	Utils::Logf("Frame Pacer init: mode %s, streamFps %d, refreshRate %.2f\n",
	            modeName, m_StreamFps, m_RefreshRate);

	m_vhsum = 0;
	m_vhcount = 0;
	m_vhidx = 0;
	std::fill(m_vhistory.begin(), m_vhistory.end(), 0);
	m_VsyncIntervalQpc = 0;
	m_LastSyncTarget = 0;
	m_ewmaVsyncDriftQpc = MsToQpc(0.0001);

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

int Pacer::getPacingMode() {
	return m_PacingMode.load(std::memory_order_acquire);
}

void Pacer::setPacingMode(int mode) {
	if (mode < 0 || mode >= PACING_MODE_COUNT) {
		mode = PACING_IMMEDIATE;
	}
	m_PacingMode.store(mode, std::memory_order_release);
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
		// IMMEDIATE and ADAPTIVE share the immediate render path; they differ only in
		// how many older frames are dropped before rendering.
		return renderModeImmediate(sceneRenderer);
	}
}

// Dequeue a new frame if available and immediately render it. When no new frame is available
// skips Present and relies on the system to continue showing the previous frame.
// Handles both PACING_IMMEDIATE (render the newest frame, drop all older) and PACING_ADAPTIVE
// (drop down to a jitter-sized target, keeping a small buffer to absorb transient spikes).
// Pros: lowest latency, output framerate matches input framerate
// Cons: only works well on Xbox Series for some reason
bool Pacer::renderModeImmediate(std::shared_ptr<VideoRenderer> &sceneRenderer) {
	const int mode = m_PacingMode.load(std::memory_order_acquire);
	int droppedToCatchUp = 0;
	AVFrame *newFrame = nullptr;

	if (mode == PACING_ADAPTIVE) {
		// Size the drop target to the measured frame-arrival jitter (adaptive-playout
		// style): on a clean link target == 1 (identical to IMMEDIATE), and it grows only
		// enough to absorb transient arrival/decoder spikes. Drop the OLDEST frames down to
		// the target, then render the oldest remaining (FIFO when buffering).
		int target = 1;
		// Leniency only helps when the source can outpace the display; otherwise a backlog
		// can't be sustained by rate, so stay strict (lowest latency).
		if (m_StreamFps >= m_RefreshRate) {
			const double frameMs = m_FrameCadence.streamPeriodMs();
			const double jitterMs = m_ArrivalJitterMs.load(std::memory_order_acquire);
			if (frameMs > 0.0) {
				// Cover the jitter TAIL (RFC 3550 J is a mean deviation), rounded to whole
				// frames; trivial jitter rounds to 0 extra -> target 1 (== IMMEDIATE).
				const double kJitterSafety = 2.0;
				int extra = static_cast<int>(((jitterMs * kJitterSafety) / frameMs) + 0.5);
				target = 1 + extra;
				if (target > 4) { // cap; FrameQueue holds ~5
					target = 4;
				}
			}
		}
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
			return false; // no frame, don't Present()
		}
	} else {
		// PACING_IMMEDIATE: lowest latency. Render the NEWEST available frame and discard
		// any older ones, so no standing buffer can accumulate. In steady state (<= 1
		// queued) the loop finds nothing extra and drops nothing.
		newFrame = FrameQueue::instance().dequeue();
		if (!newFrame) {
			return false; // no frame, don't Present()
		}
		for (AVFrame *newer; (newer = FrameQueue::instance().dequeue()) != nullptr;) {
			av_frame_free(&newFrame);
			newFrame = newer;
			++droppedToCatchUp;
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
	// PACING_ADAPTIVE to size the buffer target. Cheap; decoder thread only.
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
