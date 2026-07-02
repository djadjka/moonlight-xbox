#pragma once

#include <atomic>
#include <mutex>
#include <ppltasks.h>
#include <queue>
#include <string>
#include <vector>
#include "../Common/StepTimer.h"
#include "Pacer.h"
#include "Utils.hpp"
#include "VideoRenderer.h"

extern "C" {
#include <Limelight.h>
#include <libavcodec/avcodec.h>
#include <libavutil/hwcontext_d3d11va.h>
#include <libswscale/swscale.h>
}

#define MAX_BUFFER 1024 * 1024

typedef struct MLFrameData {
	int64_t decodeEndQpc;     // when we finished decoding
	int64_t presentTargetQpc; // timestamp when frame should be presented (slightly earlier than vsync)
	int64_t presentVsyncQpc;  // hard vsync deadline
} MLFrameData;

namespace moonlight_xbox_dx {

class FFMpegDecoder {
  public:
	// Singleton accessor
	static FFMpegDecoder &instance();

	void CompleteInitialization(const std::shared_ptr<DX::DeviceResources> &res, STREAM_CONFIGURATION *config, int pacingMode);
	int Init(int videoFormat, int width, int height, int redrawRate, void *context, int drFlags);
	void Cleanup();
	int SubmitDecodeUnit(PDECODE_UNIT decodeUnit);
	static FFMpegDecoder *getInstance();
	static DECODER_RENDERER_CALLBACKS getDecoder();
	int videoFormat, width, height, fps;
	std::recursive_mutex m_mutex;

	// Periodic stream refresh (quick menu): request an IDR every N seconds so decoder
	// drift can't accumulate on static content (issue #190). 0 = off.
	int getPeriodicRefreshSec();
	void setPeriodicRefreshSec(int seconds);

	// Bitstream dump diagnostic (quick menu): append every submitted Annex-B decode unit
	// to a timestamped file in LocalState, retrievable via the Xbox Device Portal.
	void startBitstreamDump();
	void stopBitstreamDump();
	void markBitstreamDump();  // record an "artifacts visible now" marker in a sidecar file

	// locking helper
	class LockGuard {
	  public:
		explicit LockGuard(FFMpegDecoder &ff)
		    : m_ff(ff) {
			m_ff.m_mutex.lock();
		}
		~LockGuard() {
			m_ff.m_mutex.unlock();
		}
		LockGuard(const LockGuard &) = delete;
		LockGuard &operator=(const LockGuard &) = delete;

	  private:
		FFMpegDecoder &m_ff;
	};

	[[nodiscard]] static LockGuard Lock() {
		return LockGuard(instance());
	}

  private:
	FFMpegDecoder();
	FFMpegDecoder(const FFMpegDecoder &) = delete;
	FFMpegDecoder &operator=(const FFMpegDecoder &) = delete;

	void appendBitstreamDump(const unsigned char *data, int size);
	void scheduleDumpFlushLocked(); // must be called with m_DumpMutex held
	void captureDecodedFrameSnapshot(AVFrame *hwFrame, int frameNumber); // decode thread only

	const AVCodec *decoder;
	AVCodecContext *decoder_ctx;
	AVHWDeviceContext *device_ctx;
	AVD3D11VADeviceContext *d3d11va_device_ctx;
	unsigned char *ffmpeg_buffer;
	int ffmpeg_buffer_size;
	std::shared_ptr<DX::DeviceResources> m_deviceResources;
	int m_LastFrameNumber;
	int64_t m_StreamEpochQpc;

	// Periodic stream refresh state; m_LastPeriodicIdrQpc is touched only on the decode thread
	std::atomic<int> m_PeriodicRefreshSec{0};
	int64_t m_LastPeriodicIdrQpc = 0;
	int64_t m_LastCorruptReportQpc = 0;  // rate-limits corrupt-frame logging/IDR requests

	// Bitstream dump state. The decode thread appends into m_DumpBuffer under m_DumpMutex;
	// full chunks are handed to m_DumpWriteChain (ordered background writes) so the decode
	// thread never blocks on disk I/O.
	std::atomic<bool> m_DumpEnabled{false};
	std::mutex m_DumpMutex;
	std::wstring m_DumpPath;
	std::vector<unsigned char> m_DumpBuffer;
	uint64_t m_DumpBytesTotal = 0;
	int m_DumpMarkerCount = 0;
	concurrency::task<void> m_DumpWriteChain = concurrency::task_from_result();

	// Decoded-frame snapshot burst armed by markBitstreamDump; m_SnapshotCaptured is
	// touched only on the decode thread
	std::atomic<int> m_SnapshotRemaining{0};
	int m_SnapshotCaptured = 0;
};
} // namespace moonlight_xbox_dx
