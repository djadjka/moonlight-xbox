#include "pch.h"
#include "Stats.h"
#include "Utils.hpp"
#include "../Plot/ImGuiPlots.h"
#include "../Streaming/FFMpegDecoder.h"

using namespace moonlight_xbox_dx;

Stats::Stats() :
	m_avgQueueSize(0.0),
	m_avgMbpsSmoothed(0.0)
{
	ZeroMemory(&m_ActiveWndVideoStats, sizeof(VIDEO_STATS));
	ZeroMemory(&m_LastWndVideoStats, sizeof(VIDEO_STATS));
	ZeroMemory(&m_GlobalVideoStats, sizeof(VIDEO_STATS));
}

Stats::~Stats() {
	// Persist the in-memory CSV trace to disk once, on teardown (off the render thread).
	flushCsv();
}

// Called every frame, if true is returned, the stats text is refreshed
bool Stats::ShouldUpdateDisplay(DX::StepTimer const& timer, bool isVisible, char* output, size_t length)
{
	bool shouldUpdate = false;

	if (isVisible && ImGuiPlots::instance().isEnabled()) {
		const double alpha = 0.1f;
		m_avgMbpsSmoothed = (1 - alpha) * m_avgMbpsSmoothed + alpha * m_bwTracker.GetAverageMbps();
		ImGuiPlots::instance().observeFloat(PLOT_BANDWIDTH, (float)m_avgMbpsSmoothed);
	}

	// Process stats once per second
	if (timer.GetTotalSeconds() - m_ActiveWndVideoStats.measurementStartTimestamp >= 1.0) {
		std::lock_guard<std::mutex> lock(m_mutex);

		if (isVisible) {
			// Display using data from the last 2 window periods
			VIDEO_STATS lastTwoWndStats = {};
			addVideoStats(timer, m_LastWndVideoStats, lastTwoWndStats);
			addVideoStats(timer, m_ActiveWndVideoStats, lastTwoWndStats);

			formatVideoStats(timer, lastTwoWndStats, output, length);
			shouldUpdate = true;
		}

		// CSV trace: snapshot the just-completed 1s window (computes fps) and append a
		// row, regardless of overlay visibility. Each row carries the active pacing mode.
		{
			VIDEO_STATS csvStats = {};
			addVideoStats(timer, m_ActiveWndVideoStats, csvStats);
			logCsvLine(csvStats, timer.GetTotalSeconds());
		}

		// Accumulate these values into the global stats
		addVideoStats(timer, m_ActiveWndVideoStats, m_GlobalVideoStats);

		// Move this window into the last window slot and clear it for next window
		memcpy(&m_LastWndVideoStats, &m_ActiveWndVideoStats, sizeof(VIDEO_STATS));
		ZeroMemory(&m_ActiveWndVideoStats, sizeof(VIDEO_STATS));
		m_ActiveWndVideoStats.measurementStartTimestamp = timer.GetTotalSeconds();
	}

	return shouldUpdate;
}

/// Hooks for stat producers, where possible these are combined into one call

// 1. The size in bytes of one video frame, we use this to also increment frame counters.
// 2. Time in milliseconds from first packet of a frame until fully reassembled frame is ready for decoding
//    Includes time spent in FEC reassembly
// 3. Host processing latency (encode time)
// 4. network packet loss (caller reports frame sequence number holes)
void Stats::SubmitVideoBytesAndReassemblyTime(uint32_t length, PDECODE_UNIT decodeUnit, uint32_t droppedFrames)
{
	std::lock_guard<std::mutex> lock(m_mutex);
	m_ActiveWndVideoStats.receivedFrames++;
	m_ActiveWndVideoStats.totalFrames++;

	// bandwidth
	m_bwTracker.AddBytes(length);

	// reassembly time
	uint32_t reassemblyUs = (uint32_t)(decodeUnit->enqueueTimeUs - decodeUnit->receiveTimeUs);
	m_ActiveWndVideoStats.totalReassemblyTimeUs += reassemblyUs;

	// Host processing latency
	uint16_t frameHPL = decodeUnit->frameHostProcessingLatency;
	if (frameHPL != 0) {
		if (m_ActiveWndVideoStats.minHostProcessingLatency != 0) {
			m_ActiveWndVideoStats.minHostProcessingLatency = std::min(m_ActiveWndVideoStats.minHostProcessingLatency, frameHPL);
		} else {
			m_ActiveWndVideoStats.minHostProcessingLatency = frameHPL;
		}
		m_ActiveWndVideoStats.framesWithHostProcessingLatency += 1;
		m_ActiveWndVideoStats.maxHostProcessingLatency = std::max(m_ActiveWndVideoStats.maxHostProcessingLatency, frameHPL);
		m_ActiveWndVideoStats.totalHostProcessingLatency += frameHPL;
	}

	// Network packet loss
	if (droppedFrames > 0) {
		m_ActiveWndVideoStats.networkDroppedFrames += droppedFrames;
		m_ActiveWndVideoStats.totalFrames += droppedFrames;
	}
	ImGuiPlots::instance().observeFloat(PLOT_DROPPED_NETWORK, (float)droppedFrames);

	// Host frametime graph, uses raw 90kHz units to avoid rounding errors
	static uint32_t lastHostPts = 0;
	if (lastHostPts != 0) {
		const uint32_t delta90k = (uint32_t)(decodeUnit->rtpTimestamp - lastHostPts); // wrap-safe
		ImGuiPlots::instance().observeFloat(PLOT_HOST_FRAMETIME, (float)(delta90k / 90.0f));
	}
	lastHostPts = (uint32_t)decodeUnit->rtpTimestamp;
}

// Time in milliseconds we spent decoding one frame, it is added up to later be divided by decodedFrames
void Stats::SubmitDecodeMs(double decodeMs) {
	std::lock_guard<std::mutex> lock(m_mutex);
	m_ActiveWndVideoStats.totalDecodeTime += decodeMs;
	m_ActiveWndVideoStats.decodedFrames++;
}

void Stats::SubmitDroppedFrame(int count) {
	std::lock_guard<std::mutex> lock(m_mutex);
	m_ActiveWndVideoStats.pacerDroppedFrames += count;
}

void Stats::SubmitAvgQueueSize(float avgQueueSize) {
	std::lock_guard<std::mutex> lock(m_mutex);
	m_avgQueueSize = avgQueueSize;
}

// Time in microseconds we spent in the frame pacer, and time for rendering the frame.
// Also increments the rendered frame count.
void Stats::SubmitPacerTime(int64_t pacerTimeQpc) {
	std::lock_guard<std::mutex> lock(m_mutex);
	int64_t pacerTimeUs = QpcToUs(pacerTimeQpc);
	m_ActiveWndVideoStats.totalPacerTimeUs += pacerTimeUs;
}

// Present to display latency (how close to hitting vblank we are)
void Stats::SubmitPresentPacing(double presentDisplayMs) {
	std::lock_guard<std::mutex> lock(m_mutex);
	m_ActiveWndVideoStats.totalPresentDisplayMs += presentDisplayMs;
}

// High-level render loop timings
void Stats::SubmitRenderStats(double preWaitTimeMs, double renderTimeMs, double presentTimeMs, bool hitDeadline) {
	std::lock_guard<std::mutex> lock(m_mutex);
	m_ActiveWndVideoStats.totalRenderTimeUs += static_cast<uint64_t>(renderTimeMs * 1000);
	m_ActiveWndVideoStats.renderedFrames++;

	if (hitDeadline) {
		m_ActiveWndVideoStats.hitDeadlines++;
	} else {
		m_ActiveWndVideoStats.missedDeadlines++;
	}

	// Only shown in debug builds
	m_ActiveWndVideoStats.totalPreWaitTimeUs += static_cast<uint64_t>(preWaitTimeMs * 1000);
	m_ActiveWndVideoStats.totalPresentTimeUs += static_cast<uint64_t>(presentTimeMs * 1000);
}

// On-screen interval between consecutive NEW frames; accumulate sum + sum-of-squares so
// logCsvLine can emit mean/stddev (= judder) and max per window.
void Stats::SubmitFrametime(double frametimeMs) {
	std::lock_guard<std::mutex> lock(m_mutex);
	m_ActiveWndVideoStats.frametimeCount++;
	m_ActiveWndVideoStats.totalFrametimeMs += frametimeMs;
	m_ActiveWndVideoStats.totalFrametimeMsSq += frametimeMs * frametimeMs;
	if (frametimeMs > m_ActiveWndVideoStats.maxFrametimeMs) {
		m_ActiveWndVideoStats.maxFrametimeMs = frametimeMs;
	}
}

/// private methods

void Stats::addVideoStats(DX::StepTimer const& timer, VIDEO_STATS& src, VIDEO_STATS& dst) {
	dst.receivedFrames += src.receivedFrames;
	dst.decodedFrames += src.decodedFrames;
	dst.renderedFrames += src.renderedFrames;
	dst.totalFrames += src.totalFrames;
	dst.networkDroppedFrames += src.networkDroppedFrames;
	dst.pacerDroppedFrames += src.pacerDroppedFrames;
	dst.hitDeadlines += src.hitDeadlines;
	dst.missedDeadlines += src.missedDeadlines;
	dst.frametimeCount += src.frametimeCount;
	dst.totalFrametimeMs += src.totalFrametimeMs;
	dst.totalFrametimeMsSq += src.totalFrametimeMsSq;
	dst.maxFrametimeMs = std::max(dst.maxFrametimeMs, src.maxFrametimeMs);
	dst.totalReassemblyTimeUs += src.totalReassemblyTimeUs;
	dst.totalDecodeTime += src.totalDecodeTime;
	dst.totalPacerTimeUs += src.totalPacerTimeUs;
	dst.totalRenderTimeUs += src.totalRenderTimeUs;
	dst.totalPreWaitTimeUs += src.totalPreWaitTimeUs;
	dst.totalPresentTimeUs += src.totalPresentTimeUs;
	dst.totalPresentDisplayMs += src.totalPresentDisplayMs;

	if (dst.minHostProcessingLatency == 0) {
		dst.minHostProcessingLatency = src.minHostProcessingLatency;
	}
	else if (src.minHostProcessingLatency != 0) {
		dst.minHostProcessingLatency = std::min(dst.minHostProcessingLatency, src.minHostProcessingLatency);
	}
	dst.maxHostProcessingLatency = std::max(dst.maxHostProcessingLatency, src.maxHostProcessingLatency);
	dst.totalHostProcessingLatency += src.totalHostProcessingLatency;
	dst.framesWithHostProcessingLatency += src.framesWithHostProcessingLatency;

	if (!LiGetEstimatedRttInfo(&dst.lastRtt, &dst.lastRttVariance)) {
		dst.lastRtt = 0;
		dst.lastRttVariance = 0;
	}
	else {
		// Our logic to determine if RTT is valid depends on us never
		// getting an RTT of 0. ENet currently ensures RTTs are >= 1.
		assert(dst.lastRtt > 0);
	}

	double now = timer.GetTotalSeconds();

	// Initialize the measurement start point if this is the first video stat window
	if (!dst.measurementStartTimestamp) {
		dst.measurementStartTimestamp = src.measurementStartTimestamp;
	}

	// The following code assumes the global measure was already started first
	assert(dst.measurementStartTimestamp <= src.measurementStartTimestamp);

	dst.totalFps = (double)dst.totalFrames / (now - dst.measurementStartTimestamp);
	dst.receivedFps = (double)dst.receivedFrames / (now - dst.measurementStartTimestamp);
	dst.decodedFps = (double)dst.decodedFrames / (now - dst.measurementStartTimestamp);
	dst.renderedFps = (double)dst.renderedFrames / (now - dst.measurementStartTimestamp);
}

// Append one CSV row per 1s window to LocalState\pacing_log.csv (pull via Device Portal).
// Each row is self-labelled with the active pacing mode so a session that cycles modes can
// be split per-mode for analysis. Always-on while streaming; ~1 line/sec.
void Stats::logCsvLine(VIDEO_STATS& s, double now) {
	if (!m_csvHeaderWritten) {
		m_csvBuffer.reserve(256 * 1024); // avoid reallocations during a normal session
		m_csvBuffer += "# --- session start ---\n"
		               "t_s,mode,recv_fps,dec_fps,rend_fps,frames_in_q,q_ms,render_ms,present_ms,decode_ms,net_drop_pct,pacer_drops,rtt_ms,bitrate_mbps,ft_mean_ms,ft_sd_ms,ft_max_ms,missed_pct,adaptive_target\n";
		m_csvHeaderWritten = true;
	}

	const char* mode;
	switch (Pacer::instance().getPacingMode()) {
		case Pacer::PACING_DISPLAY_LOCKED: mode = "display-locked"; break;
		case Pacer::PACING_QT:             mode = "qt";             break;
		case Pacer::PACING_ADAPTIVE:       mode = "adaptive";       break;
		case Pacer::PACING_LEGACY:         mode = "legacy";         break;
		default:                           mode = "drain";          break;
	}

	double q_ms       = s.renderedFrames ? (double) s.totalPacerTimeUs / 1000.0 / s.renderedFrames : 0.0;
	double render_ms  = s.renderedFrames ? (double) s.totalRenderTimeUs / 1000.0 / s.renderedFrames : 0.0;
	double present_ms = s.renderedFrames ? (double) s.totalPresentTimeUs / 1000.0 / s.renderedFrames : 0.0;
	double decode_ms  = s.decodedFrames ? s.totalDecodeTime / s.decodedFrames : 0.0;
	double net_drop   = s.totalFrames ? (double) s.networkDroppedFrames * 100.0 / s.totalFrames : 0.0;

	// Frametime stats (judder): mean/stddev of the on-screen interval between new frames.
	double ft_mean = s.frametimeCount ? s.totalFrametimeMs / s.frametimeCount : 0.0;
	double ft_var  = s.frametimeCount ? (s.totalFrametimeMsSq / s.frametimeCount) - (ft_mean * ft_mean) : 0.0;
	double ft_sd   = ft_var > 0.0 ? sqrt(ft_var) : 0.0;
	double missed  = (s.hitDeadlines + s.missedDeadlines)
	                     ? (double) s.missedDeadlines * 100.0 / (s.hitDeadlines + s.missedDeadlines)
	                     : 0.0;

	char buf[400];
	int n = snprintf(buf, sizeof(buf),
	                 "%.1f,%s,%.2f,%.2f,%.2f,%.2f,%.3f,%.3f,%.3f,%.3f,%.3f,%u,%u,%.1f,%.3f,%.3f,%.3f,%.2f,%d\n",
	                 now, mode, s.receivedFps, s.decodedFps, s.renderedFps,
	                 m_avgQueueSize, q_ms, render_ms, present_ms, decode_ms,
	                 net_drop, s.pacerDroppedFrames, s.lastRtt, m_bwTracker.GetAverageMbps(),
	                 ft_mean, ft_sd, s.maxFrametimeMs, missed, Pacer::instance().getAdaptiveTarget());
	if (n > 0) {
		m_csvBuffer.append(buf, n); // pure in-memory append, no syscall on the render thread
	}

	// Safety valve for pathologically long sessions only (~4 MB ≈ 11 h); never fires in a test.
	if (m_csvBuffer.size() > 4u * 1024 * 1024) {
		flushCsv();
	}
}

// Write the accumulated CSV buffer to LocalState\pacing_log.csv. Called OFF the hot path
// (on disconnect via ~Stats, or the rare safety valve). Not locked: the only in-stream
// caller (logCsvLine) already holds m_mutex, and the destructor runs with no contention.
void Stats::flushCsv() {
	if (m_csvBuffer.empty()) {
		return;
	}
	try {
		auto folder = Windows::Storage::ApplicationData::Current->LocalFolder;
		std::wstring path(folder->Path->Data());
		path += L"\\pacing_log.csv";
		std::ofstream f(path.c_str(), std::ios::app | std::ios::binary);
		if (f.is_open()) {
			f.write(m_csvBuffer.data(), (std::streamsize) m_csvBuffer.size());
			m_csvBuffer.clear();
		}
	} catch (...) {
		// best effort
	}
}

void Stats::formatVideoStats(DX::StepTimer const& timer, VIDEO_STATS& stats, char* output, size_t length) {
	FFMpegDecoder& ffmpeg = FFMpegDecoder::instance();

	int offset = 0;
	const char* codecString;
	int ret = -1;

	// Start with an empty string
	output[offset] = 0;

	switch (ffmpeg.videoFormat)
	{
	case VIDEO_FORMAT_H264:
		codecString = "H.264";
		break;

	case VIDEO_FORMAT_H264_HIGH8_444:
		codecString = "H.264 4:4:4";
		break;

	case VIDEO_FORMAT_H265:
		codecString = "HEVC";
		break;

	case VIDEO_FORMAT_H265_REXT8_444:
		codecString = "HEVC 4:4:4";
		break;

	case VIDEO_FORMAT_H265_MAIN10:
		if (LiGetCurrentHostDisplayHdrMode()) {
			codecString = "HEVC 10-bit HDR";
		}
		else {
			codecString = "HEVC 10-bit SDR";
		}
		break;

	case VIDEO_FORMAT_H265_REXT10_444:
		if (LiGetCurrentHostDisplayHdrMode()) {
			codecString = "HEVC 10-bit HDR 4:4:4";
		}
		else {
			codecString = "HEVC 10-bit SDR 4:4:4";
		}
		break;

	case VIDEO_FORMAT_AV1_MAIN8:
		codecString = "AV1";
		break;

	case VIDEO_FORMAT_AV1_HIGH8_444:
		codecString = "AV1 4:4:4";
		break;

	case VIDEO_FORMAT_AV1_MAIN10:
		if (LiGetCurrentHostDisplayHdrMode()) {
			codecString = "AV1 10-bit HDR";
		}
		else {
			codecString = "AV1 10-bit SDR";
		}
		break;

	case VIDEO_FORMAT_AV1_HIGH10_444:
		if (LiGetCurrentHostDisplayHdrMode()) {
			codecString = "AV1 10-bit HDR 4:4:4";
		}
		else {
			codecString = "AV1 10-bit SDR 4:4:4";
		}
		break;

	default:
		codecString = "UNKNOWN";
		break;
	}

	if (stats.receivedFps > 0) {
		ret = snprintf(&output[offset],
						length - offset,
						"Video stream: %dx%d %.2f FPS (%s)\n",
						ffmpeg.width,
						ffmpeg.height,
						stats.totalFps,
						codecString);
		if (ret < 0 || (size_t)ret >= (length - offset)) {
			Utils::Log("Error: stringifyVideoStats length overflow\n");
			return;
		}

		offset += ret;

		double avgVideoMbps = m_bwTracker.GetAverageMbps();
		double peakVideoMbps = m_bwTracker.GetPeakMbps();

		const char* pacingModeStr;
		switch (Pacer::instance().getPacingMode()) {
			case Pacer::PACING_DISPLAY_LOCKED: pacingModeStr = "display-locked";      break;
			case Pacer::PACING_QT:             pacingModeStr = "immediate, qt";       break;
			case Pacer::PACING_ADAPTIVE:       pacingModeStr = "immediate, adaptive"; break;
			case Pacer::PACING_LEGACY:         pacingModeStr = "immediate, legacy";   break;
			default:                           pacingModeStr = "immediate, drain";    break;
		}

		ret = snprintf(&output[offset],
					   length - offset,
					   "Bitrate: %.1f Mbps, Peak (%us): %.1f\n"
					   "Incoming frame rate from network: %.2f FPS\n"
					   "Decoding frame rate: %.2f FPS\n"
					   "Rendering frame rate: %.2f FPS (%s)\n",
					   avgVideoMbps,
					   m_bwTracker.GetWindowSeconds(),
					   peakVideoMbps,
					   stats.receivedFps,
					   stats.decodedFps,
					   stats.renderedFps,
					   pacingModeStr);
		if (ret < 0 || (size_t)ret >= (length - offset)) {
			Utils::Log("Error: stringifyVideoStats length overflow\n");
			return;
		}

		offset += ret;
	}

	if (stats.framesWithHostProcessingLatency > 0) {
		ret = snprintf(&output[offset],
					   length - offset,
					   "Host processing latency min/max/average: %.1f/%.1f/%.1f ms\n",
					   (double)stats.minHostProcessingLatency / 10,
					   (double)stats.maxHostProcessingLatency / 10,
					   (double)stats.totalHostProcessingLatency / 10 / stats.framesWithHostProcessingLatency);
		if (ret < 0 || (size_t)ret >= (length - offset)) {
			Utils::Log("Error: stringifyVideoStats length overflow\n");
			return;
		}

		offset += ret;
	}
	else {
		// If all frames are duplicates this can happen, but let's avoid having the whole stats area change height
		ret = snprintf(&output[offset],
					   length - offset,
					   "Host processing latency min/max/avg: -/-/- ms\n");
		if (ret < 0 || (size_t)ret >= (length - offset)) {
			Utils::Log("Error: stringifyVideoStats length overflow\n");
			return;
		}

		offset += ret;
	}

	if (stats.renderedFrames != 0) {
		char rttString[32];

		if (stats.lastRtt != 0) {
			snprintf(rttString, sizeof(rttString), "%u ms (variance: %u ms)", stats.lastRtt, stats.lastRttVariance);
		}
		else {
			snprintf(rttString, sizeof(rttString), "N/A");
		}

		ret = snprintf(&output[offset],
					   length - offset,
					   "Frames dropped by your network connection: %.2f%%\n"
					   "Frames dropped due to network jitter: %.2f%%\n"
					   "Average network latency: %s\n"
					   "Average reassembly/decoding time: %.2f/%.2f ms\n"
					   "Average frames in queue: %.1f\n"
					   "Average frame queue/render/present: %.2f/%.2f/%.2f ms\n",
					   stats.totalFrames ? (double)stats.networkDroppedFrames / stats.totalFrames * 100 : 0.0f,
					   stats.totalFrames ? (double)stats.pacerDroppedFrames / stats.totalFrames * 100 : 0.0f,
					   rttString,
					   stats.decodedFrames ? (double)stats.totalReassemblyTimeUs / 1000.0 / stats.decodedFrames : 0.0f,
					   stats.decodedFrames ? (double)stats.totalDecodeTime / stats.decodedFrames : 0.0f,
					   m_avgQueueSize,
					   stats.renderedFrames ? (double)stats.totalPacerTimeUs / 1000.0 / stats.renderedFrames : 0.0f,
					   stats.renderedFrames ? (double)stats.totalRenderTimeUs / 1000.0 / stats.renderedFrames : 0.0f,
					   stats.renderedFrames ? (double)stats.totalPresentTimeUs / 1000.0 / stats.renderedFrames : 0.0f);
		if (ret < 0 || (size_t)ret >= (length - offset)) {
			Utils::Log("Error: stringifyVideoStats length overflow\n");
			return;
		}

		offset += ret;
	}

#if defined(_DEBUG)
	// Developer-only stats that might be too confusing
	// If you add lines here, add more height pixels in StatsRenderer::CreateWindowSizeDependentResources()
	if (stats.renderedFrames != 0) {
		ret = snprintf(&output[offset],
					   length - offset,
					   "------\n"
					   "Missed present rate: %.2f%%\n"
					   "PreWait/Render: %.2f/%.2f ms\n",
					   stats.hitDeadlines ? ((double)stats.missedDeadlines / (stats.missedDeadlines + stats.hitDeadlines)) * 100 : 0.0f,
					   (double)stats.totalPreWaitTimeUs / 1000.0 / stats.renderedFrames,
					   (double)stats.totalRenderTimeUs / 1000.0 / stats.renderedFrames);
		if (ret < 0 || (size_t)ret >= (length - offset)) {
			Utils::Log("Error: stringifyVideoStats length overflow\n");
			return;
		}

		offset += ret;
	}
#endif
}
