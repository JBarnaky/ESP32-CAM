#include <cassert>          // assert()
#include <cstdarg>          // va_list
#include <cstddef>          // size_t
#include <cstdint>          // uint32_t, uint64_t …
#include <cstdio>           // printf(), sprintf()
#include <cstring>          // memcpy()
#include <chrono>           // high‑resolution timing
#include <iostream>        // std::cout
 /*---------------------------------------------------------------*/
 /*  Compile‑time configuration – turn the optional ESP‑32 features off   */
#define INCLUDE_PERIPH 0
#define INCLUDE_I2C    0
#define INCLUDE_MQTT   0
#define USE_MPU6050    0
#define USE_MPU9250    0
/*---------------------------------------------------------------*/
/*  Simple data structures that mimic the ESP‑32 camera API           */
struct camera_fb_t {
    uint8_t* buf;
    size_t   len;
};

/*---------------------------------------------------------------*/
/*  Global variables that the original code expects (all reduced to the
 *  minimum needed for the unit‑tests).                               */
static const size_t   FRAME_BUFFER_CAPACITY = 1024 * 1024;   // 1 MiB
static const int      MAX_STREAMS = 4;

static uint8_t        streamMemory[MAX_STREAMS][FRAME_BUFFER_CAPACITY];
static uint8_t* streamBuffer[MAX_STREAMS] = { streamMemory[0] };
static size_t         streamBufferSize[MAX_STREAMS] = { 0 };
static int            frameSemaphore[MAX_STREAMS] = { 0 };

static bool           doKeepFrame = false;
static bool           doRecording = false;
static bool           dbgMotion = false;
static bool           useMotion = false;
static bool           isCapturing = false;
static bool           forceRecord = false;
static bool           dashCamOn = false;
static bool           stopPlayback = false;
static uint32_t       maxFrameBuffSize = 1024 * 1024;   // 1 MiB
static int            vidStreams = 1;               // only stream 0 is used
static uint32_t       frameCnt = 0;
static const uint32_t frameLimit = 10;              // small limit for the demo
static uint64_t       dTimeTot = 0;
static const uint32_t FPS = 30;
static const uint32_t buzzerDuration = 0;
static const bool     buzzerUse = false;

/*---------------------------------------------------------------*/
/*  Test‑only controllable switches                                   */
static bool   simulateNoFrame = false;   // forces esp_camera_fb_get() -> nullptr
static bool   simulateMotion = false;   // value returned by checkMotion()
static size_t simulateFrameLength = 256;   // length of the fake frame data
static uint8_t fbData[FRAME_BUFFER_CAPACITY]; // backing storage for fake frames

/*---------------------------------------------------------------*/
/*  Helper / stub functions – they do nothing except record that they
 *  have been called, which is enough for the unit‑tests.               */
static uint32_t millis()
{
    using namespace std::chrono;
    static const steady_clock::time_point start = steady_clock::now();
    return static_cast<uint32_t>(duration_cast<milliseconds>(steady_clock::now() - start).count());
}

/* camera frame acquisition ------------------------------------------------*/
camera_fb_t* esp_camera_fb_get()
{
    if (simulateNoFrame) return nullptr;
    static camera_fb_t fb{ fbData, 0 };
    fb.len = simulateFrameLength;
    fb.buf = fbData;
    return &fb;
}
void esp_camera_fb_return(camera_fb_t*) { /* no‑op */ }

/* fake semaphore – does nothing on Windows --------------------------------*/
void xSemaphoreGive(int) {}

/* logging – very small wrappers around printf ------------------------------*/
void LOG_ALT(const char* fmt, ...)
{
    va_list ap;
    va_start(ap, fmt);
    vprintf(fmt, ap);
    va_end(ap);
    putchar('\n');
}
void LOG_WRN(const char* fmt, ...)
{
    va_list ap;
    va_start(ap, fmt);
    vprintf(fmt, ap);
    va_end(ap);
    putchar('\n');
}

/* other stubs ------------------------------------------------------------*/
void timeLapse(camera_fb_t*) {}
void keepFrame(camera_fb_t*) { /* count calls in the test harness */ }
bool doMonitor(bool flag) { return flag; }
bool checkMotion(camera_fb_t*, bool, bool = false) { return simulateMotion; }
bool getPIRval() { return false; }
bool checkAccelMove() { return false; }
void showProgress() {}
void saveFrame(camera_fb_t*) { ++frameCnt; }
void openAvi() {}
void closeAvi() {}
void wsAsyncSendJson(const char*, const char*) {}
void stopPlaying() {}
void logLine() {}
void setLamp(int) {}
void buzzerAlert(bool) {}
void mqttPublish(const char*) {}
void mqttPublishPath(const char*, const char*) {}

/*---------------------------------------------------------------*/
/*  The function we want to test – unchanged apart from the return‐type     */
//static bool processFrame() {
//    // get camera frame
//    static bool haveMotion = false;
//    bool res = true;
//    uint32_t dTime = millis();
//
//    camera_fb_t* fb = esp_camera_fb_get();
//    if (fb == NULL || !fb->len || fb->len > maxFrameBuffSize) return false;
//    timeLapse(fb);
//
//    for (int i = 0; i < vidStreams; i++) {
//        if (!streamBufferSize[i] && streamBuffer[i] != NULL) {
//            memcpy(streamBuffer[i], fb->buf, fb->len);
//            streamBufferSize[i] = fb->len;
//            xSemaphoreGive(frameSemaphore[i]); // signal frame ready for stream
//        }
//    }
//    if (doKeepFrame) {
//        keepFrame(fb);
//        doKeepFrame = false;
//    }
//
//    // determine if time to check for motion change
//    int reasonId = 0;
//    bool prevMotion = haveMotion;
//    if (doMonitor(doRecording ? isCapturing : dbgMotion ? false : true)) {
//        if (useMotion && checkMotion(fb, isCapturing)) reasonId = 1; // check 1 in N frames
//        if (!useMotion) checkMotion(fb, false, true); // calc light level only
//#if INCLUDE_PERIPH
//        if (pirUse && getPIRval()) reasonId = 2;
//#endif
//#if INCLUDE_I2C && (USE_MPU6050 || USE_MPU9250)
//        if (accelUse && checkAccelMove()) reasonId = 3;
//#endif
//        haveMotion = (reasonId) ? true : false;
//    }
//
//    // process motion status
//    if (haveMotion && !prevMotion) {
//        // start of movement detection
//        keepFrame(fb);
//#if INCLUDE_PERIPH
//        buzzerAlert(true); // sound buzzer if enabled
//        if (lampAuto && nightTime) setLamp(lampLevel);  // switch on lamp if requested
//#endif
//    }
//    if (!haveMotion) {
//#if INCLUDE_PERIPH
//        if (lampAuto) setLamp(0); // switch off lamp
//        buzzerAlert(false); // switch off buzzer if still on
//#endif
//    }
//
//    // recording status
//    bool prevCapture = isCapturing;
//    isCapturing = haveMotion | forceRecord;
//    if (isCapturing && !prevCapture) {
//        // new movement has occurred or record button pressed, start recording
//        stopPlaying(); // terminate any playback
//        stopPlayback = true; // stop any subsequent playback
//        if (!dashCamOn) LOG_ALT("Capture started by %s%s%s%s", reasonId == 0 ? "Button" : "", reasonId == 1 ? "Camera " : "", reasonId == 2 ? "PIR" : "", reasonId == 3 ? "Accelerometer" : "");
//#if INCLUDE_MQTT
//        if (mqtt_active) {
//            sprintf(jsonBuff, "{\"RECORD\":\"ON\", \"TIME\":\"%s\"}", esp_log_system_timestamp());
//            mqttPublish(jsonBuff);
//            mqttPublishPath("record", "on");
//        }
//#endif
//        wsAsyncSendJson("ustatus", "\"showRecord\":1");
//        openAvi();
//    }
//
//    if (isCapturing) {
//        // capture is ongoing
//        showProgress();
//        if (frameCnt < frameLimit) {
//            dTimeTot += millis() - dTime;
//            saveFrame(fb);
//            if (frameCnt == frameLimit) {
//                // stop saving frames for this avi as limit reached
//                isCapturing = forceRecord = false;
//                if (!dashCamOn) {
//                    logLine();
//                    LOG_WRN("Auto closed recording after %u frames", frameLimit);
//                }
//            }
//        }
//#if INCLUDE_PERIPH
//        if (buzzerUse && frameCnt / FPS >= buzzerDuration) buzzerAlert(false); // switch off after given period
//#endif
//    }
//
//    esp_camera_fb_return(fb);
//    if (!isCapturing && prevCapture) {
//        // finish recording (normal or forced)
//        closeAvi();
//        wsAsyncSendJson("ustatus", "\"showRecord\":0");
//        stopPlayback = false; // allow for playbacks
//    }
//    return res;
//}

// Refactor processFrame
static bool processFrame() {
    // Track motion state across frames to detect transitions (start/end)
    static bool haveMotion = false;

    // State machine to handle lifecycle events (motion/capture start/stop)
    static enum {
        STATE_NORMAL, STATE_MOTION_START, STATE_MOTION_END,
        STATE_CAPTURE_START, STATE_CAPTURE_END
    } frameState = STATE_NORMAL;

    // Record start time to measure processing duration
    const uint32_t frameStart = millis();

    // Acquire a new frame buffer from the camera driver
    camera_fb_t* fb = esp_camera_fb_get();

    // Safety check: Exit if memory allocation failed
    if (fb == nullptr) {
        return false;
    }

    // Validate frame size; release buffer immediately if invalid to prevent leaks
    if (fb->len == 0 || fb->len > maxFrameBuffSize) {
        esp_camera_fb_return(fb);
        return false;
    }

    // Cache buffer pointers for faster access in subsequent calls
    const uint8_t* frameData = fb->buf;
    const size_t frameLen = fb->len;

    // Process time-lapse logic (e.g., skip frames or save interval shots)
    timeLapse(fb);

    // Copy frame to streaming buffers if they are empty and ready for new data
    for (int i = 0; i < vidStreams; ++i) {
        if (streamBufferSize[i] == 0 && streamBuffer[i] != nullptr) {
            memcpy(streamBuffer[i], frameData, frameLen);
            streamBufferSize[i] = frameLen;
            // Signal the streaming task that new data is available
            xSemaphoreGive(frameSemaphore[i]);
        }
    }

    // Handle manual "keep frame" request (e.g., from user button press)
    if (doKeepFrame) {
        keepFrame(fb);
        doKeepFrame = false;
    }

    int reasonId = 0;
    const bool prevMotion = haveMotion;
    // Determine if motion detection should run based on recording/debug flags
    const bool monitorArg = doRecording ? isCapturing : (dbgMotion ? false : true);

    // Run motion detection sensors (Camera, PIR, Accelerometer) if enabled
    if (doMonitor(monitorArg)) {
        if (useMotion && checkMotion(fb, isCapturing)) reasonId = 1; // Camera motion
        if (!useMotion) checkMotion(fb, false, true); // Update background model only
#if INCLUDE_PERIPH
        if (pirUse && getPIRval()) reasonId = 2;       // PIR sensor trigger
#endif
#if INCLUDE_I2C && USE_MPU
        if (accelUse && checkAccelMove()) reasonId = 3; // Accelerometer trigger
#endif
        // Update global motion state based on any active sensor trigger
        haveMotion = (reasonId != 0);
    }

    // Default state is normal; update only if transitions occur
    frameState = STATE_NORMAL;

    // Detect transition between motion and no-motion states
    if (haveMotion != prevMotion) {
        frameState = haveMotion ? STATE_MOTION_START : STATE_MOTION_END;
    }

    // Detect transition between recording and idle states
    if (isCapturing != (haveMotion || forceRecord)) {
        frameState = (haveMotion || forceRecord) ? STATE_CAPTURE_START : STATE_CAPTURE_END;
    }

    // Handle actions triggered specifically by the start or end of motion
    switch (frameState) {
    case STATE_MOTION_START:
        keepFrame(fb); // Save the first frame where motion was detected
#if INCLUDE_PERIPH
        buzzerAlert(true); // Activate alarm
        if (lampAuto && nightTime) setLamp(lampLevel); // Turn on light if dark
#endif
        break;
    case STATE_MOTION_END:
#if INCLUDE_PERIPH
        if (lampAuto) setLamp(0); // Turn off light
        buzzerAlert(false); // Deactivate alarm
#endif
        break;
    default:
        break;
    }

    const bool prevCapture = isCapturing;
    // Update recording state: active if motion detected or forced manually
    isCapturing = haveMotion || forceRecord;

    // Handle actions triggered specifically by the start or stop of recording
    switch (frameState) {
    case STATE_CAPTURE_START:
        stopPlaying(); // Halt any ongoing playback to free resources
        stopPlayback = true;
        // Log the source that triggered the recording
        if (!dashCamOn) LOG_ALT("Capture started by %s%s%s%s",
            reasonId == 0 ? "Button" : "", reasonId == 1 ? "Camera " : "",
            reasonId == 2 ? "PIR" : "", reasonId == 3 ? "Accelerometer" : "");
#if INCLUDE_MQTT
        if (mqtt_active) {
            // Notify remote systems via MQTT that recording has begun
            snprintf(jsonBuff, sizeof(jsonBuff), "{\"RECORD\":\"ON\", \"TIME\":\"%s\"}",
                esp_log_system_timestamp());
            mqttPublish(jsonBuff);
            mqttPublishPath("record", "on");
        }
#endif
        wsAsyncSendJson("ustatus", "\"showRecord\":1"); // Update UI status
        openAvi(); // Initialize new AVI file for writing
        break;
    default:
        break;
    }

    // If currently recording, save frames and manage limits
    if (isCapturing) {
        showProgress(); // Update LED or UI progress indicator
        if (frameCnt < frameLimit) {
            uint32_t elapsed = millis() - frameStart;
            dTimeTot += elapsed; // Accumulate processing time for stats
            saveFrame(fb);       // Write frame to AVI file

            // Check if maximum frame count reached
            if (frameCnt == frameLimit) {
                isCapturing = false;
                forceRecord = false;
                if (!dashCamOn) {
                    logLine();
                    LOG_WRN("Auto closed recording after %u frames", frameLimit);
                }
            }
        }
#if INCLUDE_PERIPH
        // Turn off buzzer after configured duration expires
        if (buzzerUse && frameCnt >= buzzerDuration * FPS) buzzerAlert(false);
#endif
    }

    // Release camera buffer back to driver (MUST be called before exit)
    esp_camera_fb_return(fb);

    // Finalize recording if capture just ended
    if (frameState == STATE_CAPTURE_END) {
        closeAvi();      // Finalize AVI file header and close
        wsAsyncSendJson("ustatus", "\"showRecord\":0"); // Update UI status
        stopPlayback = false; // Allow playback again
    }

    return true;
}



/*---------------------------------------------------------------*/
/*  Simple test‑harness – no external framework                         */
static int  keepFrameCalls = 0;   // incremented by the overridden keepFrame()
static int  openAviCalls = 0;
static int  closeAviCalls = 0;

/* reset all globals to a known baseline before each test               */
static void reset_state()
{
    simulateNoFrame = false;
    simulateMotion = false;
    simulateFrameLength = 256;
    maxFrameBuffSize = 1024 * 1024;
    doKeepFrame = false;
    doRecording = false;
    dbgMotion = false;
    useMotion = false;
    isCapturing = false;
    forceRecord = false;
    dashCamOn = false;
    stopPlayback = false;
    frameCnt = 0;
    dTimeTot = 0;
    keepFrameCalls = 0;
    openAviCalls = 0;
    closeAviCalls = 0;

    for (int i = 0; i < MAX_STREAMS; ++i) {
        streamBufferSize[i] = 0;
        std::memset(streamMemory[i], 0, FRAME_BUFFER_CAPACITY);
    }

    /* fill the fake camera buffer with a known pattern – useful for the
     * stream‑copy test.                                                */
    for (size_t i = 0; i < sizeof(fbData); ++i) fbData[i] = static_cast<uint8_t>(i & 0xFF);
}

/* ----------------------------------------------------------------- */
static void test_no_frame()
{
    reset_state();
    simulateNoFrame = true;                     // make esp_camera_fb_get() return nullptr
    bool ok = processFrame();
    assert(!ok && "processFrame must return false when there is no frame");
    assert(streamBufferSize[0] == 0 && "Stream buffer must stay empty");
    std::cout << "  test_no_frame … ok\n";
}

/* ----------------------------------------------------------------- */
static void test_no_motion()
{
    reset_state();
    simulateNoFrame = false;
    simulateMotion = false;   // checkMotion() reports “no motion”
    useMotion = true;    // enable the motion check in the code
    bool ok = processFrame();

    assert(ok && "processFrame must succeed when a frame is present");
    assert(isCapturing == false && "Recording must NOT start when motion is false");
    assert(openAviCalls == 0 && "openAvi must not be called");
    assert(streamBufferSize[0] == simulateFrameLength && "frame must be copied to stream buffer");
    /* verify that the copied data matches the source pattern */
    for (size_t i = 0; i < simulateFrameLength; ++i) {
        assert(streamBuffer[0][i] == fbData[i] && "stream buffer content mismatch");
    }
    std::cout << "  test_no_motion … ok\n";
}

/* ----------------------------------------------------------------- */
static void test_with_motion()
{
    reset_state();
    simulateNoFrame = false;
    simulateMotion = true;   // checkMotion() reports motion
    useMotion = true;  // enable motion detection
    bool ok = processFrame();

    assert(ok && "processFrame must succeed with a frame");
    assert(isCapturing, "Recording must start when motion is detected");
    assert(openAviCalls == 1 && "openAvi should be called exactly once");
    assert(frameCnt == 1 && "One frame must be saved on the first call");
    assert(keepFrameCalls == 1 && "keepFrame must be called once for the motion start");
    std::cout << "  test_with_motion … ok\n";
}

/*---------------------------------------------------------------*/
/*  Very small benchmark – reports average time per call in µs       */
static void run_benchmark()
{
    constexpr size_t ITERATIONS = 10'000'000;   // enough for a stable measurement

    reset_state();
    /* minimal work per iteration – no motion, no frame‑copy side‑effects */
    useMotion = false;
    simulateNoFrame = false;
    simulateFrameLength = 256;               // any size < maxFrameBuffSize

    std::cout << "\nRunning benchmark (" << ITERATIONS << " calls)…\n";

    auto t0 = std::chrono::high_resolution_clock::now();
    for (size_t i = 0; i < ITERATIONS; ++i) {
        (void)processFrame();                // result is ignored – we only measure cost
    }
    auto t1 = std::chrono::high_resolution_clock::now();
    std::chrono::duration<double> sec = t1 - t0;

    double us_per_call = (sec.count() * 1e6) / ITERATIONS;
    std::cout << "  Total time   : " << sec.count() << " s\n";
    std::cout << "  Avg per call : " << us_per_call << " µs\n";
}

/*---------------------------------------------------------------*/
int main()
{
    std::cout << "=== processFrame unit-tests ===\n";

    test_no_frame();
    test_no_motion();
    test_with_motion();

    std::cout << "\nAll unit-tests passed.\n";

    run_benchmark();

    return 0;
}
