#include "pch.h"
#include "FFMpegDecoder.h"
#include "../Plot/ImGuiPlots.h"
#include "StatsRenderer.h"

#include <Common\DirectXHelper.h>
#include <d3d11_1.h>
#include <fstream>
#include "Utils.hpp"
#include "moonlight_xbox_dxMain.h"
#include <gamingdeviceinformation.h>

extern "C" {
#include "Limelight.h"
#include <third_party\h264bitstream\h264_stream.h>
#include <libavcodec/avcodec.h>
#include <libswscale/swscale.h>
#include <libavutil/hwcontext_d3d11va.h>
#include <libavutil/time.h>
}

using namespace moonlight_xbox_dx;

#define INITIAL_DECODER_BUFFER_SIZE (256 * 1024)

// Bitstream dump: flush to disk every 4 MiB, hard-stop at 2 GiB so a forgotten
// dump can't fill the console's storage.
#define DUMP_FLUSH_THRESHOLD (4 * 1024 * 1024)
#define DUMP_MAX_BYTES (2ULL * 1024 * 1024 * 1024)

// Decoded-frame snapshots: each "Mark" press captures this many CONSECUTIVE decoded
// frames (consecutive so offline analysis can attribute the error added in a single
// frame to the coding tools of that exact frame), capped per session.
#define SNAPSHOT_BURST_FRAMES 4
#define SNAPSHOT_MAX_FRAMES 40

static bool ensure_buf_size(unsigned char **buf, int *buf_size, int required_size)
{
	if (*buf_size >= required_size)
		return true;

	FQLog("ensure_buf_size grew from %d -> %d\n", *buf_size, required_size);

	*buf_size = required_size;
	*buf = (unsigned char *)realloc(*buf, *buf_size);
	if (!*buf) {
		return false;
	}

	return true;
}

namespace moonlight_xbox_dx {
	FFMpegDecoder &FFMpegDecoder::instance() {
		static FFMpegDecoder inst;
		return inst;
	}

	FFMpegDecoder::FFMpegDecoder():
		width(0),
		height(0),
		videoFormat(0),
		decoder(nullptr),
		decoder_ctx(nullptr),
		device_ctx(nullptr),
		d3d11va_device_ctx(nullptr),
		ffmpeg_buffer(nullptr),
		ffmpeg_buffer_size(0),
		m_deviceResources(nullptr),
		m_LastFrameNumber(0) {
	}

	void lock_context(void *user) {
		auto me = (FFMpegDecoder*)user;
		me->m_mutex.lock();
	}

	void unlock_context(void *user) {
		auto me = (FFMpegDecoder*)user;
		me->m_mutex.unlock();
	}

	void ffmpeg_log_callback(void *ptr, int level, const char *fmt, va_list vl) {
		char lineBuffer[1024];
		static int printPrefix = 1;

		if ((level & 0xFF) > av_log_get_level()) {
			return;
		}

		// We need to use the *previous* printPrefix value to determine whether to
		// print the prefix this time. av_log_format_line() will set the printPrefix
		// value to indicate whether the prefix should be printed *next time*.
		bool shouldPrefixThisMessage = printPrefix != 0;

		av_log_format_line(ptr, level, fmt, vl, lineBuffer, sizeof(lineBuffer), &printPrefix);
		Utils::Logf(shouldPrefixThisMessage ? "[ffmpeg] %s" : "%s", lineBuffer);
	}

    void FFMpegDecoder::CompleteInitialization(const std::shared_ptr<DX::DeviceResources>& res, STREAM_CONFIGURATION *config, int pacingMode) {
		this->m_deviceResources = res;
		this->fps = config->fps;
		Pacer::instance().init(res, config->fps, res->GetRefreshRate(), pacingMode);
	}

	int FFMpegDecoder::Init(int videoFormat, int width, int height, int redrawRate, void* context, int drFlags) {
		this->videoFormat = videoFormat;
		this->width = width;
		this->height = height;
		this->fps = 60; // correctly set in CompleteInitialization

		this->m_LastFrameNumber = 0;
		this->ffmpeg_buffer_size = 0;
		this->m_StreamEpochQpc = 0;
		this->m_LastPeriodicIdrQpc = 0;
		this->m_LastCorruptReportQpc = 0;


#if LIBAVCODEC_VERSION_INT < AV_VERSION_INT(58,10,100)
		avcodec_register_all();
#endif

		av_log_set_level(AV_LOG_ERROR);

		av_log_set_callback(&ffmpeg_log_callback);
#pragma warning(suppress : 4996)

		if (videoFormat & VIDEO_FORMAT_MASK_H264) {
			decoder = avcodec_find_decoder(AV_CODEC_ID_H264);
			Utils::Log("Using H264\n");
		}
		else if (videoFormat & VIDEO_FORMAT_MASK_H265) {
			decoder = avcodec_find_decoder(AV_CODEC_ID_HEVC);
			Utils::Log("Using HEVC\n");
		}

		if (decoder == NULL) {
			Utils::Log("Couldn't find decoder\n");
			return -1;
		}

		decoder_ctx = avcodec_alloc_context3(decoder);
		if (decoder_ctx == NULL) {
			Utils::Log("Couldn't allocate context\n");
			return -1;
		}
		decoder_ctx->opaque = this;

		AVBufferRef* hw_device_ctx = av_hwdevice_ctx_alloc(AV_HWDEVICE_TYPE_D3D11VA);
		device_ctx = reinterpret_cast<AVHWDeviceContext*>(hw_device_ctx->data);
		d3d11va_device_ctx = reinterpret_cast<AVD3D11VADeviceContext*>(device_ctx->hwctx);
		d3d11va_device_ctx->device = m_deviceResources->GetD3DDevice();
		d3d11va_device_ctx->device_context = m_deviceResources->GetD3DDeviceContext();
		d3d11va_device_ctx->lock = lock_context;
		d3d11va_device_ctx->unlock = unlock_context;
		d3d11va_device_ctx->lock_ctx = this;
		int err2;
		if ((err2 = av_hwdevice_ctx_init(hw_device_ctx)) < 0) {
			Utils::Logf("Failed to create specified DirectX Video device: %d\n", err2);
			Cleanup();
			return err2;
		}

		decoder_ctx->hw_device_ctx = av_buffer_ref(hw_device_ctx);
		av_buffer_unref(&hw_device_ctx);
		decoder_ctx->pix_fmt = AV_PIX_FMT_D3D11;
		decoder_ctx->sw_pix_fmt = (videoFormat & VIDEO_FORMAT_MASK_10BIT) ? AV_PIX_FMT_P010 : AV_PIX_FMT_NV12;
		decoder_ctx->pkt_timebase.num = 1;
		decoder_ctx->pkt_timebase.den = 90000;
		decoder_ctx->width = width;
		decoder_ctx->height = height;

		int err = avcodec_open2(decoder_ctx, decoder, NULL);
		if (err < 0) {
			char msg[2048];
			sprintf(msg, "Failed to create FFMpeg Codec: %d\n", err);
			Utils::Log(msg);
			return err;
		}

		if (decoder_ctx->pix_fmt != AV_PIX_FMT_D3D11) {
    		Utils::Log("Warning: decoder did not select AV_PIX_FMT_D3D11\n");
		}

		if (!ensure_buf_size(&ffmpeg_buffer, &ffmpeg_buffer_size, INITIAL_DECODER_BUFFER_SIZE + AV_INPUT_BUFFER_PADDING_SIZE)) {
			Utils::Log("Couldn't allocate initial ffmpeg_buffer\n");
			Cleanup();
			return -1;
		}

		return 0;
	}

	void FFMpegDecoder::Cleanup() {
		// Flush and close a still-running bitstream dump before the stream goes away
		stopBitstreamDump();

		avcodec_free_context(&decoder_ctx);
		if (ffmpeg_buffer != NULL) {
			free(ffmpeg_buffer);
			ffmpeg_buffer = NULL;
			ffmpeg_buffer_size = 0;
		}
		m_LastFrameNumber = 0;

		Pacer::instance().deinit();

		Utils::Log("FFMpegDecoder::Cleanup\n");
	}

    static inline int frame_attach_userdata(AVFrame *frame, int64_t decodeEndQpc) {
	    if (!frame) return AVERROR(EINVAL);

	    if (frame->opaque_ref) {
		    av_buffer_unref(&frame->opaque_ref);
	    }

	    AVBufferRef *buf = av_buffer_allocz(sizeof(MLFrameData));
	    if (!buf) return AVERROR(ENOMEM);

	    MLFrameData *data = (MLFrameData *)buf->data;
	    data->decodeEndQpc = decodeEndQpc;
	    frame->opaque_ref = buf;

	    return 0;
    }

    // Called by the VideoDec thread
	int FFMpegDecoder::SubmitDecodeUnit(PDECODE_UNIT decodeUnit) {
		LARGE_INTEGER decodeStart, decodeEnd;
		PLENTRY entry = decodeUnit->bufferList;
		int length = 0;
		QueryPerformanceCounter(&decodeStart);
		decodeEnd.QuadPart = 0; // only set when a frame actually comes out of the decoder

		if (m_StreamEpochQpc == 0) m_StreamEpochQpc = decodeStart.QuadPart;

		// Periodic stream refresh: re-anchor the stream with a host IDR every N seconds so
		// decoder drift can't accumulate on static content (issue #190). Any IDR that
		// arrives on its own (host-side refresh, loss recovery) restarts the interval,
		// so this acts as a watchdog rather than a fixed-rate requester.
		int refreshSec = m_PeriodicRefreshSec.load(std::memory_order_acquire);
		if (refreshSec > 0) {
			if (m_LastPeriodicIdrQpc == 0 || decodeUnit->frameType == FRAME_TYPE_IDR) {
				m_LastPeriodicIdrQpc = decodeStart.QuadPart;
			} else if (QpcToMs(decodeStart.QuadPart - m_LastPeriodicIdrQpc) >= refreshSec * 1000.0) {
				m_LastPeriodicIdrQpc = decodeStart.QuadPart;
				LiRequestIdrFrame();
				Utils::Log("Periodic stream refresh: requested IDR\n");
			}
		} else {
			m_LastPeriodicIdrQpc = 0;
		}

		if (!ensure_buf_size(&ffmpeg_buffer, &ffmpeg_buffer_size, decodeUnit->fullLength + AV_INPUT_BUFFER_PADDING_SIZE)) {
			Utils::Logf("Couldn't realloc ffmpeg_buffer\n");
			return DR_NEED_IDR;
		}

	    while (entry != NULL) {
		    memcpy(ffmpeg_buffer + length, entry->data, entry->length);
		    length += entry->length;
		    entry = entry->next;
	    }
		memset(ffmpeg_buffer + length, 0, AV_INPUT_BUFFER_PADDING_SIZE);

		// Diagnostic: capture the exact Annex-B bytes we are about to feed the decoder
		if (m_DumpEnabled.load(std::memory_order_acquire)) {
			appendBitstreamDump(ffmpeg_buffer, length);
		}

		// Detect breaks in the frame sequence indicating dropped packets
		uint32_t droppedFramesNetwork = 0;
		if (m_LastFrameNumber > 0 && decodeUnit->frameNumber > (m_LastFrameNumber + 1)) {
			// Any frame number greater than m_LastFrameNumber + 1 represents a dropped frame
			droppedFramesNetwork = decodeUnit->frameNumber - (m_LastFrameNumber + 1);
		}
		m_LastFrameNumber = decodeUnit->frameNumber;

		if (!decodeUnit->rtpTimestamp) {
			// Estimate for hosts that don't send timestamps (e.g. Wolf)
			LogOnce("Warning: host is not sending RTP timestamps, this may hurt frame pacing\n");
			double ptsMs = QpcToMs(QpcNow() - m_StreamEpochQpc);
			decodeUnit->rtpTimestamp = (uint32_t)(ptsMs * 90.0);
			decodeUnit->presentationTimeUs = (uint64_t)(ptsMs * 1000.0);
		}

		// track stats for a variety of things we can track at the same time
		m_deviceResources->GetStats()->SubmitVideoBytesAndReassemblyTime(length, decodeUnit, droppedFramesNetwork);

		// ffmpeg_decode
		AVPacket *pkt = av_packet_alloc();
		pkt->data = ffmpeg_buffer;
		pkt->size = length;
		pkt->pts = (int64_t)decodeUnit->rtpTimestamp;
		pkt->dts = pkt->pts;

		int err = avcodec_send_packet(decoder_ctx, pkt);
		av_packet_unref(pkt);
		av_packet_free(&pkt);
		if (err < 0) {
			char ffmpegError[1024];
			av_strerror(err, ffmpegError, 1024);
			Utils::Logf("avcodec_send_packet failed: %s\n", ffmpegError);
			return DR_NEED_IDR;
		}

		bool sawCorruptFrame = false;

		while (err >= 0) {
			AVFrame* frame = av_frame_alloc();
			err = avcodec_receive_frame(decoder_ctx, frame);
			if (err == AVERROR(EAGAIN) || err == AVERROR_EOF) {
				av_frame_free(&frame);
				break;
			}
			else if (err < 0) {
				char ffmpegError[1024];
				av_strerror(err, ffmpegError, sizeof(ffmpegError));
				Utils::Logf("avcodec_receive_frame failed: %s\n", ffmpegError);
				av_frame_free(&frame);
				return DR_NEED_IDR;
			}

			// The hardware decoder normally conceals decode problems silently; surface them
			// and re-anchor with an IDR instead of letting the corruption linger on screen.
			// Rate-limited to once per second so a decoder that (wrongly) flags every frame
			// can't trigger an IDR storm that would tank the stream.
			if (frame->decode_error_flags != 0 || (frame->flags & AV_FRAME_FLAG_CORRUPT)) {
				if (m_LastCorruptReportQpc == 0 || QpcToMs(decodeStart.QuadPart - m_LastCorruptReportQpc) >= 1000.0) {
					m_LastCorruptReportQpc = decodeStart.QuadPart;
					Utils::Logf("Decoder flagged corrupt output (decode_error_flags=0x%x), requesting IDR\n",
						frame->decode_error_flags);
					sawCorruptFrame = true;
				}
			}

			// Capture a frame timestamp to measuring pacing delay
			QueryPerformanceCounter(&decodeEnd);
			frame_attach_userdata(frame, decodeEnd.QuadPart);

			// Feed the adaptive pacer this frame's decode time, EXCLUDING IDR frames (intra
			// frames are large and slow by nature -> a periodic IDR would falsely pin the
			// buffer). Sustained P-frame decode pressure is the real "complex scene" signal.
			if (frame->pict_type != AV_PICTURE_TYPE_I) {
				Pacer::instance().observeDecodeMs(QpcToMs(decodeEnd.QuadPart - decodeStart.QuadPart));
			}

			FQLog("✓ Frame decoded [pts: %.3fms] [in#: %d] [out#: %d] [lost: %d] decode time %.3fms\n",
				frame->pts / 90.0,
				decodeUnit->frameNumber, decoder_ctx->frame_num,
				decodeUnit->frameNumber - decoder_ctx->frame_num,
				QpcToMs(decodeEnd.QuadPart - decodeStart.QuadPart));

			// Diagnostic: after a "Mark" press, capture the hardware decoder's ACTUAL output
			// for a few consecutive frames so it can be compared offline against a reference
			// decode of the same bitstream (isolates decoder drift and fingerprints the
			// coding tool responsible). Must happen before Pacer takes ownership.
			if (m_SnapshotRemaining.load(std::memory_order_acquire) > 0) {
				m_SnapshotRemaining.fetch_sub(1, std::memory_order_acq_rel);
				captureDecodedFrameSnapshot(frame, decodeUnit->frameNumber);
			}

			// Queue the frame for rendering. frame is now owned by Pacer.
			Pacer::instance().submitFrame(frame);

			// Even though we have a valid frame, the ffmpeg API needs us to loop and call avcodec_receive_frame()
			// again where we expect to get AVERROR(EAGAIN) and break out.
		}

		double decodeTimeMs = QpcToMs(decodeEnd.QuadPart - decodeStart.QuadPart);
		if (decodeEnd.QuadPart > decodeStart.QuadPart) {
			m_deviceResources->GetStats()->SubmitDecodeMs(decodeTimeMs);
		}

		// Not the best way to handle this. BUT IT DOES FIX XBOX ONE TEARING!!!!
		// Honestly this did take too much time of my life (and AndyG life too) to care to make a better version
		// If you want to fix this, have fun! (And hopefully you have Microsoft blessing/tools/support for that)
		// if (IsXboxOne()) {
		// 	float remainingMs = (1000.0f / fps) - decodeTimeMs - 2.0; // 2ms buffer time
		// 	if (remainingMs > 0.0) {
		// 		//Utils::Logf("SubmitDecodeUnit sleeping %.3fms\n", remainingMs);
		// 		SleepUntilQpc(QpcNow() + MsToQpc(remainingMs));
		// 	}
		// }

		return sawCorruptFrame ? DR_NEED_IDR : DR_OK;
	}

	int FFMpegDecoder::getPeriodicRefreshSec() {
		return m_PeriodicRefreshSec.load(std::memory_order_acquire);
	}

	void FFMpegDecoder::setPeriodicRefreshSec(int seconds) {
		m_PeriodicRefreshSec.store(seconds, std::memory_order_release);
		Utils::Logf("Periodic stream refresh set to %ds\n", seconds);
	}

	// Begin a fresh bitstream dump. Each Start writes to a NEW timestamped file in LocalState
	// (pull it via the Device Portal). We immediately ask the host for an IDR so the dump
	// begins with parameter sets + a keyframe and is decodable offline from the first frame.
	void FFMpegDecoder::startBitstreamDump() {
		std::lock_guard<std::mutex> lock(m_DumpMutex);
		if (m_DumpEnabled.load(std::memory_order_acquire)) {
			return;
		}
		SYSTEMTIME st;
		GetLocalTime(&st);
		wchar_t name[80];
		swprintf(name, 80, L"bitstream_%04d%02d%02d_%02d%02d%02d.%s",
		         st.wYear, st.wMonth, st.wDay, st.wHour, st.wMinute, st.wSecond,
		         (videoFormat & VIDEO_FORMAT_MASK_H264) ? L"h264" : L"h265");
		try {
			auto folder = Windows::Storage::ApplicationData::Current->LocalFolder;
			m_DumpPath = std::wstring(folder->Path->Data()) + L"\\" + name;
			std::ofstream f(m_DumpPath.c_str(), std::ios::trunc | std::ios::binary);
			if (!f.is_open()) {
				m_DumpPath.clear();
				return;
			}
		} catch (...) {
			m_DumpPath.clear();
			return;
		}
		m_DumpBytesTotal = 0;
		m_DumpMarkerCount = 0;
		m_SnapshotCaptured = 0;
		m_DumpBuffer.clear();
		m_DumpBuffer.reserve(DUMP_FLUSH_THRESHOLD + INITIAL_DECODER_BUFFER_SIZE);
		m_DumpEnabled.store(true, std::memory_order_release);
		LiRequestIdrFrame();
		Utils::Log("Bitstream dump started\n");
	}

	// Record an "artifacts are visible NOW" marker: appends the current dump byte offset
	// and frame number to a sidecar <dump>.markers.txt, so offline analysis knows exactly
	// which stretch of the bitstream to render without needing precisely timed photos.
	// Called from the UI thread; the tiny synchronous append is fine there.
	void FFMpegDecoder::markBitstreamDump() {
		std::lock_guard<std::mutex> lock(m_DumpMutex);
		if (m_DumpPath.empty()) {
			Utils::Log("Dump marker ignored: no bitstream dump running\n");
			return;
		}
		m_DumpMarkerCount++;
		SYSTEMTIME st;
		GetLocalTime(&st);
		char line[160];
		snprintf(line, sizeof(line), "marker %d local_time=%02d:%02d:%02d byte_offset=%llu last_frame=%d stream_ms=%.0f\n",
		         m_DumpMarkerCount, st.wHour, st.wMinute, st.wSecond,
		         (unsigned long long)m_DumpBytesTotal, m_LastFrameNumber,
		         QpcToMs(QpcNow() - m_StreamEpochQpc));
		try {
			std::ofstream f((m_DumpPath + L".markers.txt").c_str(), std::ios::app);
			if (f.is_open()) {
				f << line;
			}
		} catch (...) {
			// best effort
		}
		// Arm a burst of decoded-frame snapshots for the frames that follow this marker
		if (m_DumpEnabled.load(std::memory_order_acquire)) {
			m_SnapshotRemaining.store(SNAPSHOT_BURST_FRAMES, std::memory_order_release);
		}
		Utils::Logf("Dump marker %d recorded at byte %llu\n", m_DumpMarkerCount, (unsigned long long)m_DumpBytesTotal);
	}

	// Transfer one decoded frame from the GPU and append it (small header + packed
	// Y/UV planes) to <dump>.frames.bin through the background write chain. ~25 MB
	// per 4K HDR frame, so a burst briefly stalls the decode thread; acceptable for
	// a manually-triggered diagnostic. Called on the decode thread only.
	void FFMpegDecoder::captureDecodedFrameSnapshot(AVFrame *hwFrame, int frameNumber) {
		if (m_SnapshotCaptured >= SNAPSHOT_MAX_FRAMES) {
			return;
		}
		AVFrame *sw = av_frame_alloc();
		if (sw == NULL) {
			return;
		}
		int err = av_hwframe_transfer_data(sw, hwFrame, 0);
		if (err != 0) {
			char errorstring[512];
			av_strerror(err, errorstring, sizeof(errorstring));
			Utils::Logf("Snapshot: hwframe transfer failed: %s\n", errorstring);
			av_frame_free(&sw);
			return;
		}
		int width = sw->width, height = sw->height, format = sw->format;
		// NV12 (8-bit) / P010 (10-bit) layout: full-res Y plane + half-height interleaved UV
		size_t bps = (format == AV_PIX_FMT_P010LE) ? 2 : 1;
		size_t yRow = (size_t)width * bps;
		size_t uvRows = (size_t)height / 2;
		uint64_t dumpBytes;
		{
			std::lock_guard<std::mutex> lock(m_DumpMutex);
			dumpBytes = m_DumpBytesTotal;
		}
		// 48-byte little-endian header so the offline tool can find and identify each frame
		uint32_t header[12] = {0};
		memcpy(&header[0], "MXFR", 4);
		header[1] = 1;  // version
		header[2] = (uint32_t)frameNumber;
		header[3] = (uint32_t)width;
		header[4] = (uint32_t)height;
		header[5] = (uint32_t)format;  // AVPixelFormat enum value
		memcpy(&header[6], &dumpBytes, 8);
		header[8] = (uint32_t)(yRow * height + yRow * uvRows);  // payload size
		std::vector<unsigned char> chunk;
		chunk.reserve(sizeof(header) + yRow * height + yRow * uvRows);
		chunk.insert(chunk.end(), (unsigned char *)header, (unsigned char *)header + sizeof(header));
		for (int r = 0; r < height; r++) {
			const unsigned char *row = sw->data[0] + (size_t)r * sw->linesize[0];
			chunk.insert(chunk.end(), row, row + yRow);
		}
		for (size_t r = 0; r < uvRows; r++) {
			const unsigned char *row = sw->data[1] + r * sw->linesize[1];
			chunk.insert(chunk.end(), row, row + yRow);
		}
		av_frame_free(&sw);
		{
			std::lock_guard<std::mutex> lock(m_DumpMutex);
			if (m_DumpPath.empty()) {
				return;
			}
			auto data = std::make_shared<std::vector<unsigned char>>(std::move(chunk));
			std::wstring path = m_DumpPath + L".frames.bin";
			m_DumpWriteChain = m_DumpWriteChain.then([path, data]() {
				try {
					std::ofstream f(path.c_str(), std::ios::app | std::ios::binary);
					if (f.is_open()) {
						f.write((const char *)data->data(), data->size());
					}
				} catch (...) {
					// best effort
				}
			});
		}
		m_SnapshotCaptured++;
		Utils::Logf("Snapshot: captured decoded frame %d (%dx%d fmt %d, %d/%d)\n",
		            frameNumber, width, height, format, m_SnapshotCaptured, SNAPSHOT_MAX_FRAMES);
	}

	// End the current dump: flush whatever is buffered and forget the file.
	void FFMpegDecoder::stopBitstreamDump() {
		std::lock_guard<std::mutex> lock(m_DumpMutex);
		if (!m_DumpEnabled.load(std::memory_order_acquire) && m_DumpPath.empty()) {
			return;
		}
		m_DumpEnabled.store(false, std::memory_order_release);
		m_SnapshotRemaining.store(0, std::memory_order_release);
		scheduleDumpFlushLocked();
		m_DumpPath.clear();
		Utils::Logf("Bitstream dump stopped (%llu bytes)\n", (unsigned long long)m_DumpBytesTotal);
	}

	// Called on the decode thread for every submitted decode unit while dumping.
	// Only pays for a memcpy; disk writes happen on the background write chain.
	void FFMpegDecoder::appendBitstreamDump(const unsigned char *data, int size) {
		std::lock_guard<std::mutex> lock(m_DumpMutex);
		if (m_DumpPath.empty() || size <= 0) {
			return;
		}
		if (m_DumpBytesTotal + (uint64_t)size > DUMP_MAX_BYTES) {
			m_DumpEnabled.store(false, std::memory_order_release);
			scheduleDumpFlushLocked();
			m_DumpPath.clear();
			Utils::Log("Bitstream dump reached the 2 GiB cap, stopping\n");
			return;
		}
		m_DumpBuffer.insert(m_DumpBuffer.end(), data, data + size);
		m_DumpBytesTotal += (uint64_t)size;
		if (m_DumpBuffer.size() >= DUMP_FLUSH_THRESHOLD) {
			scheduleDumpFlushLocked();
		}
	}

	// Hand the accumulated chunk to a background task. Chained on the previous write so
	// chunks land in the file in order; same best-effort ofstream append as the CSV logger.
	void FFMpegDecoder::scheduleDumpFlushLocked() {
		if (m_DumpBuffer.empty() || m_DumpPath.empty()) {
			return;
		}
		auto chunk = std::make_shared<std::vector<unsigned char>>(std::move(m_DumpBuffer));
		m_DumpBuffer = std::vector<unsigned char>();
		m_DumpBuffer.reserve(DUMP_FLUSH_THRESHOLD + INITIAL_DECODER_BUFFER_SIZE);
		std::wstring path = m_DumpPath;
		m_DumpWriteChain = m_DumpWriteChain.then([path, chunk]() {
			try {
				std::ofstream f(path.c_str(), std::ios::app | std::ios::binary);
				if (f.is_open()) {
					f.write((const char *)chunk->data(), chunk->size());
				}
			} catch (...) {
				// best effort
			}
		});
	}

	//Helpers
	int initCallback(int videoFormat, int width, int height, int redrawRate, void* context, int drFlags) noexcept {
		return FFMpegDecoder::instance().Init(videoFormat, width, height, redrawRate, context, drFlags);
	}

	void cleanupCallback()noexcept {
		FFMpegDecoder::instance().Cleanup();
	}

	int submitDecodeUnit(PDECODE_UNIT decodeUnit) noexcept {
		return FFMpegDecoder::instance().SubmitDecodeUnit(decodeUnit);
	}

	DECODER_RENDERER_CALLBACKS FFMpegDecoder::getDecoder() {
		DECODER_RENDERER_CALLBACKS decoder_callbacks_sdl;
		LiInitializeVideoCallbacks(&decoder_callbacks_sdl);
		decoder_callbacks_sdl.setup = initCallback;
		decoder_callbacks_sdl.cleanup = cleanupCallback;
		decoder_callbacks_sdl.submitDecodeUnit = submitDecodeUnit;
		decoder_callbacks_sdl.capabilities = CAPABILITY_DIRECT_SUBMIT | CAPABILITY_INTRA_REFRESH;
		//decoder_callbacks_sdl.capabilities = CAPABILITY_DIRECT_SUBMIT | CAPABILITY_REFERENCE_FRAME_INVALIDATION_HEVC;
		return decoder_callbacks_sdl;
	}
}
