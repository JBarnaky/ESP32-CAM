#include <cstdint>
#include <cstdio>
#include <cstring>
#include <cmath>
#include <algorithm>
#include <chrono>
#include <thread>
#include <vector>
#include <string>
#include <iostream>
#include <mutex>
#include <functional>

// ---------------------------------------------------------------------------
// Feature toggles – match the original code but force them to 0 so the
// implementation stays fully self‑contained.
// ---------------------------------------------------------------------------
#define INCLUDE_AUDIO      0
#define INCLUDE_MQTT       0
#define INCLUDE_TELEM      0
#define INCLUDE_SMTP       0
#define INCLUDE_TGRAM      0
#define INCLUDE_FTP_HFS    0

// ---------------------------------------------------------------------------
// Simple logging macros (replace the original LOG_* macros)
// ---------------------------------------------------------------------------
#define LOG_VRB(...) do { printf("[VRB] "); printf(__VA_ARGS__); printf("\n"); } while(0)
#define LOG_WRN(...) do { printf("[WRN] "); printf(__VA_ARGS__); printf("\n"); } while(0)
#define LOG_INF(...) do { printf("[INF] "); printf(__VA_ARGS__); printf("\n"); } while(0)
#define LOG_ALT(...) do { printf("[ALT] "); printf(__VA_ARGS__); printf("\n"); } while(0)

inline void logLine() { printf("\n"); }

// ---------------------------------------------------------------------------
// Constants used by the original code
// ---------------------------------------------------------------------------
constexpr int FILE_NAME_LEN = 256;
constexpr int AVI_HEADER_LEN = 128;
constexpr int RAMSIZE = 4096;
constexpr char AVI_EXT[] = "avi";
constexpr char AVITEMP[] = "temp_avi.avi";
constexpr int SeekSet = 0;          // beginning of the file (compatible with Arduino)

// ---------------------------------------------------------------------------
// Global state needed by closeAvi()
// ---------------------------------------------------------------------------
static uint32_t startTime = 0;               // When recording started (ms)
static uint32_t minSeconds = 2;               // Minimum acceptable duration (seconds)
static uint32_t frameCnt = 30;              // Fake number of frames
static uint32_t fsizePtr = 0;               // Index into frameData[]
static uint32_t cTime = 0;               // Temporary timestamp holder
static uint32_t vidSize = 1024 * 1024;     // Fake file size (1 MiB)
static uint32_t dTimeTot = 0;               // Dummy aggregated timing values
static uint32_t fTimeTot = 0;
static uint32_t wTimeTot = 1;
static uint32_t oTime = 0;
static uint32_t FPS = 30;              // Required FPS (only logged)
static bool haveSrt = false;
static bool dashCamOn = false;
static bool forceRecord = false;
static bool doRecording = false;
static bool mqtt_active = false;

static char partName[64] = "PART";
static char aviFileName[FILE_NAME_LEN];
static char jsonBuff[256];
static char hostName[32] = "host";

static uint8_t iSDbuffer[RAMSIZE];               // Temp buffer used while writing the file
static size_t  highPoint = 0;                   // How many bytes in iSDbuffer are valid

static uint8_t aviHeader[AVI_HEADER_LEN];       // Header written at the start of the file

// ---------------------------------------------------------------------------
// Mock timer – deterministic for unit‑tests
// ---------------------------------------------------------------------------
static uint32_t g_millis = 0;
static uint32_t millis() { return g_millis; }
static void set_millis(uint32_t v) { g_millis = v; }

// ---------------------------------------------------------------------------
// Stub for a file that mimics the behaviour used by closeAvi()
// ---------------------------------------------------------------------------
class StubFile {
public:
    std::vector<char> data;   // All data written to the file
    bool isOpen = false;
    size_t pos = 0;

    // The real firmware opens the file elsewhere – we only need a ready‑to‑write object.
    void open(const char* /*name*/) { data.clear(); pos = 0; isOpen = true; }

    void write(const void* buf, size_t len) {
        if (!isOpen) return;
        const char* p = static_cast<const char*>(buf);
        if (pos + len > data.size()) data.resize(pos + len);
        std::copy(p, p + len, data.begin() + pos);
        pos += len;
    }

    void seek(int offset, int mode) {
        if (!isOpen) return;
        if (mode == SeekSet) pos = static_cast<size_t>(offset);
        else                 pos = static_cast<size_t>(offset);
    }

    void close() { isOpen = false; }

    bool closed() const { return !isOpen; }
};
static StubFile aviFile;

// ---------------------------------------------------------------------------
// Stub storage abstraction (rename / remove)
// ---------------------------------------------------------------------------
struct StubStorage {
    std::string renamedFrom;
    std::string renamedTo;
    std::string removedFile;

    void rename(const char* src, const char* dst) {
        renamedFrom = src ? src : "";
        renamedTo = dst ? dst : "";
    }
    void remove(const char* fname) { removedFile = fname ? fname : ""; }
} STORAGE;

// ---------------------------------------------------------------------------
// Miscellaneous stubs matching the original API
// ---------------------------------------------------------------------------
struct FrameInfo { char frameSizeStr[16]; };
static FrameInfo frameData[4] = { {"1080p"} };

static void finalizeAviIndex(uint32_t) { /* no‑op */ }
static size_t writeAviIndex(uint8_t*, size_t) { return 0; }
static void buildAviHdr(uint8_t fps, uint32_t, uint32_t) {
    // Store the calculated FPS in the first byte – simple way to verify it later.
    std::memset(aviHeader, 0, AVI_HEADER_LEN);
    aviHeader[0] = fps;
}
static void stopTelemetry(const char*) { /* no‑op */ }
static void checkMemory() { /* no‑op */ }
static bool checkFreeStorage() { return true; }
static const char* fmtSize(uint32_t sz) {
    static char buf[32];
    snprintf(buf, sizeof(buf), "%u B", sz);
    return buf;
}
static void tgramAlert(const char*, const char*) { /* no‑op */ }
static void emailAlert(const char*, const char*) { /* no‑op */ }
static void mqttPublish(const char*) { /* no‑op */ }
static void mqttPublishPath(const char*, const char*) { /* no‑op */ }
static const char* esp_log_system_timestamp() { return "2022-01-01T00:00:00Z"; }
static void finishAudioRecord(bool) { /* no‑op */ }
static size_t writeWavFile(uint8_t*, size_t) { return 0; }
static bool haveWavFile() { return false; }

static std::mutex aviMutex;                         // Used by xSemaphore* wrappers
static void xSemaphoreTake(std::mutex& m, uint32_t) { m.lock(); }
static void xSemaphoreGive(std::mutex& m) { m.unlock(); }
static constexpr uint32_t portMAX_DELAY = 0xFFFFFFFF;

// ---------------------------------------------------------------------------
// The function under test (only the #if blocks are guarded by the macros above)
// ---------------------------------------------------------------------------
//static bool closeAvi() {
//    // closes the recorded file
//    uint32_t vidDuration = millis() - startTime;
//    uint32_t vidDurationSecs = lround(vidDuration / 1000.0);
//    logLine();
//    LOG_VRB("Capture time %u, min seconds: %u ", vidDurationSecs, minSeconds);
//
//    cTime = millis();
//    // write remaining frame content to SD
//    aviFile.write(iSDbuffer, highPoint);
//    size_t readLen = 0;
//    bool haveWav = false;
//#if INCLUDE_AUDIO
//    // add wav file if exists
//    finishAudioRecord(true);
//    haveWav = haveWavFile();
//    if (haveWav) {
//        do {
//            readLen = writeWavFile(iSDbuffer, RAMSIZE);
//            aviFile.write(iSDbuffer, readLen);
//        } while (readLen > 0);
//    }
//#endif
//    // save avi index
//    finalizeAviIndex(frameCnt);
//    do {
//        readLen = writeAviIndex(iSDbuffer, RAMSIZE);
//        if (readLen) aviFile.write(iSDbuffer, readLen);
//    } while (readLen > 0);
//    // save avi header at start of file
//    float actualFPS = (1000.0f * (float)frameCnt) / ((float)vidDuration);
//    uint8_t actualFPSint = (uint8_t)(lround(actualFPS));
//    xSemaphoreTake(aviMutex, portMAX_DELAY);
//    buildAviHdr(actualFPSint, fsizePtr, frameCnt);
//    xSemaphoreGive(aviMutex);
//    aviFile.seek(0, SeekSet); // start of file
//    aviFile.write(aviHeader, AVI_HEADER_LEN);
//    aviFile.close();
//    LOG_VRB("Final SD storage time %lu ms", millis() - cTime);
//    uint32_t hTime = millis();
//#if INCLUDE_MQTT
//    if (mqtt_active) {
//        sprintf(jsonBuff, "{\"RECORD\":\"OFF\", \"TIME\":\"%s\"}", esp_log_system_timestamp());
//        mqttPublish(jsonBuff);
//        mqttPublishPath("record", "off");
//    }
//#endif
//    if (vidDurationSecs >= minSeconds) {
//        // name file to include actual dateTime, FPS, duration, and frame count
//        int alen = snprintf(aviFileName, FILE_NAME_LEN - 1, "%s_%s_%u_%lu%s%s%s.%s",
//            partName, frameData[fsizePtr].frameSizeStr, actualFPSint, vidDurationSecs,
//            haveWav ? "_S" : "", haveSrt ? "_M" : "", dashCamOn ? "_C" : "", AVI_EXT);
//        if (alen > FILE_NAME_LEN - 1) LOG_WRN("file name truncated");
//        STORAGE.rename(AVITEMP, aviFileName);
//        LOG_VRB("AVI close time %lu ms", millis() - hTime);
//        cTime = millis() - cTime;
//#if INCLUDE_TELEM
//        stopTelemetry(aviFileName);
//#endif
//        if (dashCamOn) forceRecord = true; // restart continuous recording
//        else {
//            // AVI stats
//            LOG_INF("******** AVI recording stats ********");
//            LOG_ALT("Recorded %s", aviFileName);
//            LOG_INF("AVI duration: %u secs", vidDurationSecs);
//            LOG_INF("Number of frames: %u", frameCnt);
//            LOG_INF("Required FPS: %u", FPS);
//            LOG_INF("Actual FPS: %0.1f", actualFPS);
//            LOG_INF("File size: %s", fmtSize(vidSize));
//            if (frameCnt) {
//                LOG_INF("Average frame length: %u bytes", vidSize / frameCnt);
//                LOG_INF("Average frame monitoring time: %u ms", dTimeTot / frameCnt);
//                LOG_INF("Average frame buffering time: %u ms", fTimeTot / frameCnt);
//                LOG_INF("Average frame storage time: %u ms", wTimeTot / frameCnt);
//            }
//            LOG_INF("Average SD write speed: %u kB/s", ((vidSize / wTimeTot) * 1000) / 1024);
//            LOG_INF("File open / completion times: %u ms / %u ms", oTime, cTime);
//            LOG_INF("Busy: %u%%", std::min(100 * (wTimeTot + fTimeTot + dTimeTot + oTime + cTime) / vidDuration, (uint32_t)100));
//            checkMemory();
//            LOG_INF("*************************************");
//            // send out notification of motion if requested
//#if INCLUDE_SMTP
//            if (smtpUse) {
//                // send email with movement image
//                char subjectMsg[50];
//                snprintf(subjectMsg, sizeof(subjectMsg) - 1, "from %s, in %s", hostName, aviFileName);
//                emailAlert("Motion Alert", subjectMsg);
//            }
//#endif
//#if INCLUDE_TGRAM
//            tgramAlert(aviFileName, "");
//#endif
//#if INCLUDE_FTP_HFS
//            if (autoUpload) {
//                if (deleteAfter) {
//                    // issue #380 - in case other files failed to transfer, do whole parent folder
//                    dateFormat(partName, sizeof(partName), true);
//                    fsStartTransfer(partName);
//                }
//                else fsStartTransfer(aviFileName); // transfer this file to remote ftp server
//            }
//#endif
//        }
//        if (!checkFreeStorage()) doRecording = forceRecord = false;
//        return true;
//    }
//    else {
//        // delete too small files if exist
//        STORAGE.remove(AVITEMP);
//        LOG_INF("Insufficient capture duration: %u secs", vidDurationSecs);
//        return false;
//    }
//}

// Fixed Point 8.8 closeAvi Refactor
static bool closeAvi() {
    // Get current time and calculate total recording duration in seconds
    const uint32_t now = millis();
    const uint32_t vidDuration = now - startTime;
    const uint32_t vidDurationSecs = (vidDuration + 500) / 1000;
    constexpr uint8_t SHIFT8 = 8;

    // Warn if recording approached timer rollover limit (~49 days)
    if (vidDuration > 49710000) {
        LOG_WRN("Timer rollover detected - recording duration may be inaccurate");
    }

    logLine();
    LOG_VRB("Capture time %u, min seconds: %u ", vidDurationSecs, minSeconds);

    // Flush any remaining frame buffer data to SD card
    const uint32_t sdStartTime = now;
    aviFile.write(iSDbuffer, highPoint);

    // Write buffered WAV audio data to file (if audio enabled)
    bool haveWav = false;
#if INCLUDE_AUDIO
    finishAudioRecord(true);
    haveWav = haveWavFile();
    if (haveWav) {
        size_t readLen = 0;
        do {
            readLen = writeWavFile(iSDbuffer, RAMSIZE);
            if (readLen) aviFile.write(iSDbuffer, readLen);
        } while (readLen > 0);
    }
#endif

    // Write AVI index table to enable video playback seeking
    finalizeAviIndex(frameCnt);
    size_t readLen = 0;
    do {
        readLen = writeAviIndex(iSDbuffer, RAMSIZE);
        if (readLen) aviFile.write(iSDbuffer, readLen);
    } while (readLen > 0);

    // Calculate actual FPS using 32-bit math (fallback to 64-bit for long recordings)
    uint8_t actualFPSint = 0;
    uint16_t fpsFixed88 = 0;
    if (vidDuration > 0) {
        if (frameCnt < 16777) {
            uint32_t num = frameCnt * 256000UL;
            fpsFixed88 = (uint16_t)(num / vidDuration);
        }
        else {
            uint64_t num = (uint64_t)frameCnt * 256000ULL;
            fpsFixed88 = (uint16_t)(num / vidDuration);
        }
        actualFPSint = (fpsFixed88 + 128) >> SHIFT8;
    }

    // Protect AVI header construction with mutex for thread safety
    xSemaphoreTake(aviMutex, portMAX_DELAY);
    buildAviHdr(actualFPSint, fsizePtr, frameCnt);
    xSemaphoreGive(aviMutex);

    // Overwrite file start with completed AVI header and close file handle
    aviFile.seek(0, SeekSet);
    aviFile.write(aviHeader, AVI_HEADER_LEN);
    aviFile.close();

    const uint32_t sdWriteTime = millis() - sdStartTime;
    LOG_VRB("Final SD storage time %lu ms", sdWriteTime);

    // Publish recording-stopped status to MQTT broker (if enabled)
    const uint32_t mqttStartTime = millis();
#if INCLUDE_MQTT
    if (mqtt_active) {
        snprintf(jsonBuff, JSON_BUFF_SIZE, "{\"RECORD\":\"OFF\", \"TIME\":\"%s\"}",
            esp_log_system_timestamp());
        mqttPublish(jsonBuff);
        mqttPublishPath("record", "off");
    }
#endif

    // Rename temp file to final name if recording met minimum duration requirement
    bool success = false;
    if (vidDurationSecs >= minSeconds) {
        // Build filename with FPS, duration, and feature flags (audio/subtitles/dashcam)
        const char* sSuffix = haveWav ? "_S" : "";
        const char* mSuffix = haveSrt ? "_M" : "";
        const char* cSuffix = dashCamOn ? "_C" : "";

        int alen = snprintf(aviFileName, FILE_NAME_LEN - 1, "%s_%s_%u_%lu%s%s%s.%s",
            partName, frameData[fsizePtr].frameSizeStr, actualFPSint, vidDurationSecs,
            sSuffix, mSuffix, cSuffix, AVI_EXT);
        if (alen > FILE_NAME_LEN - 1) LOG_WRN("file name truncated");

        STORAGE.rename(AVITEMP, aviFileName);

        const uint32_t closeElapsedTime = millis() - mqttStartTime;
        LOG_VRB("AVI close time %lu ms", closeElapsedTime);

#if INCLUDE_TELEM
        stopTelemetry(aviFileName);
#endif

        // Auto-restart recording if dashcam mode is enabled
        if (dashCamOn) forceRecord = true;
        else {
            // Log comprehensive recording statistics for performance analysis
            LOG_INF("******** AVI recording stats ********");
            LOG_ALT("Recorded %s", aviFileName);
            LOG_INF("AVI duration: %u secs", vidDurationSecs);
            LOG_INF("Number of frames: %u", frameCnt);
            LOG_INF("Required FPS: %u", FPS);
            uint8_t fpsIntPart = fpsFixed88 >> SHIFT8;
            uint8_t fpsFracPart = ((fpsFixed88 & 0xFF) * 10) >> SHIFT8;
            LOG_INF("Actual FPS: %u.%u", fpsIntPart, fpsFracPart);
            LOG_INF("File size: %s", fmtSize(vidSize));
            if (frameCnt) {
                LOG_INF("Average frame length: %u bytes", (uint32_t)(vidSize / frameCnt));
                LOG_INF("Average frame monitoring time: %u ms", dTimeTot / frameCnt);
                LOG_INF("Average frame buffering time: %u ms", fTimeTot / frameCnt);
                LOG_INF("Average frame storage time: %u ms", wTimeTot / frameCnt);
            }
            uint32_t avgSpeed = 0;
            if (wTimeTot > 0) {
                avgSpeed = (uint32_t)((vidSize * 1000ULL) / (wTimeTot * 1024ULL));
            }
            LOG_INF("Average SD write speed: %u kB/s", avgSpeed);
            LOG_INF("File open / completion times: %u ms / %u ms", oTime, sdWriteTime);
            uint32_t totalTimeSpent = wTimeTot + fTimeTot + dTimeTot + oTime + sdWriteTime;
            uint8_t busyPercent = 0;
            if (vidDuration > 0) {
                busyPercent = (uint8_t)((100ULL * totalTimeSpent) / vidDuration);
                if (busyPercent > 100) busyPercent = 100;
            }
            LOG_INF("Busy: %u%%", busyPercent);
            checkMemory();
            LOG_INF("*************************************");

            // Send motion alert notifications via configured channels
#if INCLUDE_SMTP
            if (smtpUse) {
                char subjectMsg[50];
                snprintf(subjectMsg, sizeof(subjectMsg) - 1, "from %s, in %s", hostName, aviFileName);
                emailAlert("Motion Alert", subjectMsg);
            }
#endif
#if INCLUDE_TGRAM
            tgramAlert(aviFileName, "");
#endif
#if INCLUDE_FTP_HFS
            if (autoUpload) {
                if (deleteAfter) {
                    dateFormat(partName, sizeof(partName), true);
                    fsStartTransfer(partName);
                }
                else fsStartTransfer(aviFileName);
            }
#endif
        }
        // Disable recording if storage is critically low
        if (!checkFreeStorage()) doRecording = forceRecord = false;
        success = true;
    }
    else {
        // Delete recording if it was shorter than minimum required duration
        STORAGE.remove(AVITEMP);
        LOG_INF("Insufficient capture duration: %u secs", vidDurationSecs);
        success = false;
    }
    return success;
}



// ---------------------------------------------------------------------------
// Minimal test harness (no external framework)
// ---------------------------------------------------------------------------
#define ASSERT_TRUE(expr) \
    do { ++g_testCount; if (!(expr)) { ++g_testFailures; std::cerr << "ASSERT_TRUE failed: " \
          << #expr << " (" << __FILE__ << ":" << __LINE__ << ")\n"; } } while(0)

#define ASSERT_FALSE(expr) ASSERT_TRUE(!(expr))

#define ASSERT_EQ(val1, val2) \
    do { ++g_testCount; if ((val1) != (val2)) { ++g_testFailures; std::cerr << "ASSERT_EQ failed: " \
          << #val1 << " != " << #val2 << " (" << (val1) << " vs " << (val2) << ") (" \
          << __FILE__ << ":" << __LINE__ << ")\n"; } } while(0)

static int g_testCount = 0;
static int g_testFailures = 0;

// Helper to bring the whole environment back to a clean state before each test.
static void resetGlobals() {
    startTime = 0;
    cTime = 0;
    std::memset(iSDbuffer, 0, sizeof(iSDbuffer));
    highPoint = 0;
    aviFile.open("dummy");                 // marks the file as open
    STORAGE.renamedFrom.clear();
    STORAGE.renamedTo.clear();
    STORAGE.removedFile.clear();
    haveSrt = false;
    dashCamOn = false;
    forceRecord = false;
    doRecording = false;
    set_millis(0);
}

// ---------------------------------------------------------------------------
// Test 1 – normal capture (duration >= minSeconds)
// ---------------------------------------------------------------------------
static void test_successful_close() {
    resetGlobals();

    // Simulate a 3 s recording: minSeconds = 2, so the result must be true.
    startTime = 0;
    set_millis(3000);               // 3000 ms → 3 seconds

    bool result = closeAvi();

    // ---- Expectations ----------------------------------------------------
    ASSERT_TRUE(result);                                 // should succeed
    ASSERT_TRUE(aviFile.closed());                       // file closed
    ASSERT_EQ(STORAGE.renamedFrom, AVITEMP);             // source must be temp file
    ASSERT_TRUE(STORAGE.removedFile.empty());            // remove must *not* be called
    // Expected FPS = (1000 * frameCnt) / duration = (1000*30)/3000 = 10 → stored in header[0]
    ASSERT_EQ(aviHeader[0], static_cast<uint8_t>(10));
    // Header must be written at offset 0 of the file.
    ASSERT_TRUE(aviFile.data.size() >= AVI_HEADER_LEN);
    ASSERT_EQ(static_cast<uint8_t>(aviFile.data[0]), aviHeader[0]);
}

// ---------------------------------------------------------------------------
// Test 2 – capture too short (duration < minSeconds)
// ---------------------------------------------------------------------------
static void test_insufficient_duration() {
    resetGlobals();

    // Simulate a 1.2 s recording → rounded to 1 s, below the 2‑second threshold.
    startTime = 0;
    set_millis(1200);               // 1200 ms

    bool result = closeAvi();

    // ---- Expectations ----------------------------------------------------
    ASSERT_FALSE(result);                               // must report failure
    ASSERT_TRUE(aviFile.closed());                      // file still closed
    ASSERT_TRUE(STORAGE.removedFile == AVITEMP);        // temp file must be removed
    ASSERT_TRUE(STORAGE.renamedFrom.empty());           // no rename performed
}

// ---------------------------------------------------------------------------
// Very small benchmark – average time of the “happy‑path” case
// ---------------------------------------------------------------------------
static void benchmark_closeAvi(int iterations = 1000) {
    using Clock = std::chrono::high_resolution_clock;
    uint64_t totalNs = 0;

    resetGlobals();                     // constant config for all runs
    startTime = 0;

    for (int i = 0; i < iterations; ++i) {
        aviFile.open("dummy");         // reopen the file for this iteration
        set_millis(3000);              // 3 s duration – same as test_successful_close

        auto t0 = Clock::now();
        (void)closeAvi();               // ignore return value for timing
        auto t1 = Clock::now();

        totalNs += std::chrono::duration_cast<std::chrono::nanoseconds>(t1 - t0).count();

        // Reset the file content for the next loop but keep the same global vars.
        aviFile = StubFile();          // fresh empty object (closed)
    }

    double avgUs = (totalNs / static_cast<double>(iterations)) / 1000.0;
    std::cout << "\nBenchmark: " << iterations << " iterations, avg = "
        << avgUs << " µs per closeAvi()\n";
}

// ---------------------------------------------------------------------------
// Main – runs the unit tests and, if they all pass, the benchmark.
// ---------------------------------------------------------------------------
int main() {
    std::cout << "Running unit tests for closeAvi()...\n";

    test_successful_close();
    test_insufficient_duration();

    std::cout << "\nUnit-test summary: " << g_testCount << " checks, "
        << g_testFailures << " failures.\n";

    if (g_testFailures == 0) {
        std::cout << "All tests passed – starting benchmark.\n";
        benchmark_closeAvi(500);   // 5 000 runs for a smoother average
    }
    else {
        std::cerr << "Some tests failed – benchmark skipped.\n";
        return 1;
    }
    return 0;
}
