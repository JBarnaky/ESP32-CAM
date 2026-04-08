#include <cstdint>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <cstdarg>
#include <cmath>
#include <algorithm>
#include <chrono>
#include <iostream>
#include <vector>
#include <malloc.h>                 // _aligned_malloc / _aligned_free (MSVC)

using std::max;                    // make plain max() usable in the source

/* ----------------------------------------------------------------
 *  Configuration macros – all optional ESP‑IDF features are disabled
 * ---------------------------------------------------------------- */
#ifndef INCLUDE_NEW_JPG
#define INCLUDE_NEW_JPG 0
#endif
#ifndef INCLUDE_TINYML
#define INCLUDE_TINYML 0
#endif
#ifndef INCLUDE_MQTT
#define INCLUDE_MQTT 0
#endif
#ifndef INCLUDE_HASIO
#define INCLUDE_HASIO 0
#endif
#ifndef ESP_ARDUINO_VERSION
#define ESP_ARDUINO_VERSION 0
#endif
#define ESP_ARDUINO_VERSION_VAL(major, minor, patch) ((major)*10000 + (minor)*100 + (patch))

 /* ----------------------------------------------------------------
  *  Constants that the original code expects
  * ---------------------------------------------------------------- */
#define FRAMESIZE_SXGA   0
#define RGB888_BYTES     3
#define GRAYSCALE_BYTES  1
#define RESIZE_DIM       32
#define RESIZE_DIM_SQ    (RESIZE_DIM*RESIZE_DIM)
#define JPEG_QUAL        75
#define PIXFORMAT_RGB888 0
#define MALLOC_CAP_SPIRAM 0

  /* ----------------------------------------------------------------
   *  Very small platform stubs
   * ---------------------------------------------------------------- */
static uint32_t start_ms = 0;
uint32_t millis()
{
    using namespace std::chrono;
    if (start_ms == 0) {
        start_ms = (uint32_t)duration_cast<milliseconds>(steady_clock::now().time_since_epoch()).count();
    }
    auto now = steady_clock::now();
    return (uint32_t)duration_cast<milliseconds>(now.time_since_epoch()).count() - start_ms;
}

/* memory helpers --------------------------------------------------- */
void* ps_malloc(size_t size) { return std::malloc(size); }
void* heap_caps_aligned_calloc(size_t alignment, size_t num, size_t size, int /*caps*/)
{
    size_t total = num * size;
    const size_t need = RESIZE_DIM_SQ * RGB888_BYTES;      // guarantee enough for our tests
    if (total < need) total = need;
    void* ptr = _aligned_malloc(total, (unsigned)alignment);
    if (ptr) std::memset(ptr, 0, total);
    return ptr;
}

/* logging ---------------------------------------------------------- */
void LOG_VRB(const char* fmt, ...)
{
    va_list ap; va_start(ap, fmt);
    vprintf(fmt, ap); printf("\n");
    va_end(ap);
}
void LOG_WRN(const char* fmt, ...)
{
    va_list ap; va_start(ap, fmt);
    vprintf(fmt, ap); printf("\n");
    va_end(ap);
}

/* JPEG / image helpers (stubs) -------------------------------------- */
bool jpg2rgb(const uint8_t* src, size_t len, uint8_t* dst, uint8_t /*scaling*/)
{
    size_t copy = std::min(len, (size_t)(RESIZE_DIM_SQ * RGB888_BYTES));
    std::memcpy(dst, src, copy);
    if (copy < (size_t)(RESIZE_DIM_SQ * RGB888_BYTES))
        std::memset(dst + copy, 0, (size_t)(RESIZE_DIM_SQ * RGB888_BYTES) - copy);
    return true;
}
void rescaleImage(const uint8_t* src, int /*srcW*/, int /*srcH*/,
    uint8_t* dst, int /*dstW*/, int /*dstH*/)
{
    size_t bytes = RESIZE_DIM_SQ * RGB888_BYTES;
    std::memcpy(dst, src, bytes);
}

/* fmt2jpg – used only when INCLUDE_NEW_JPG == 0 ------------------- */
bool fmt2jpg(const uint8_t* src, size_t src_len,
    int /*w*/, int /*h*/, int /*fmt*/, int /*quality*/,
    uint8_t** out_buf, size_t* out_len)
{
    *out_buf = (uint8_t*)std::malloc(src_len);
    if (!*out_buf) return false;
    std::memcpy(*out_buf, src, src_len);
    *out_len = src_len;
    return true;
}

/* other stubs ------------------------------------------------------- */
bool isNight(int) { return false; }
void checkMemory() {}
int  xSemaphoreGive(void*) { return 0; }
void mqttPublish(const char*) {}
void mqttPublishPath(const char*, const char*) {}
bool tinyMLclassify() { return true; }
char* esp_log_system_timestamp() { return (char*)"00:00:00"; }

/* ----------------------------------------------------------------
 *  Global variables that the motion routine uses
 * ---------------------------------------------------------------- */
uint8_t fsizePtr = FRAMESIZE_SXGA;          // current frame‑size index
int     colorDepth = RGB888_BYTES;          // bytes per pixel
int     stride = 0;                         // set when resolution changes

int detectStartBand = 1;
int detectEndBand = 1;
int detectNumBands = 1;
int detectChangeThreshold = 1;
int detectMotionFrames = 1;
int motionVal = 0;
int nightSwitch = 0;
bool nightTime = false;
bool dbgMotion = false;
bool dbgVerbose = false;
uint8_t lightLevel = 0;

uint8_t* motionJpeg = nullptr;   // debug JPEG buffer
size_t   motionJpegLen = 0;
uint8_t* currBuff = nullptr;     // current resized bitmap
void* motionSemaphore = nullptr;
char jsonBuff[256];

struct FrameInfo {
    uint16_t frameWidth;
    uint16_t frameHeight;
    uint8_t  scaleFactor;
    uint16_t sampleRate;
};
FrameInfo frameData[1];           // only the SXGA entry is needed

void init_globals()
{
    frameData[FRAMESIZE_SXGA].frameWidth = RESIZE_DIM;
    frameData[FRAMESIZE_SXGA].frameHeight = RESIZE_DIM;
    frameData[FRAMESIZE_SXGA].scaleFactor = 0;
    frameData[FRAMESIZE_SXGA].sampleRate = 1;
}

/* ----------------------------------------------------------------
 *  Camera frame description used by the original function
 * ---------------------------------------------------------------- */
typedef struct {
    size_t   len;
    uint8_t* buf;
    int      width;
    int      height;
} camera_fb_t;

/* ----------------------------------------------------------------
 *  ==== ORIGINAL checkMotion() – DO NOT MODIFY ====================
 * ---------------------------------------------------------------- */
//bool checkMotion(camera_fb_t* fb, bool motionStatus, bool lightLevelOnly) {
//    // check difference between current and previous image (subtract background)
//    // convert image from JPEG to downscaled RGB888 or 8 bit grayscale bitmap
//    if (fsizePtr > FRAMESIZE_SXGA) return false;
//    uint32_t dTime = millis();
//    uint32_t lux = 0;
//    static uint32_t motionCnt = 0;
//    static uint8_t fsizePtrPrev = 255; // initially invalid to force setup on first call
//    static uint8_t scaling, downsize;
//    static uint16_t reducer;
//    static int sampleWidth = 0, sampleHeight = 0;
//    static uint8_t* rgbBuf = (uint8_t*)heap_caps_aligned_calloc(16, 1, frameData[FRAMESIZE_SXGA].frameWidth * frameData[FRAMESIZE_SXGA].frameHeight * RGB888_BYTES / 8, MALLOC_CAP_SPIRAM); // must be 16 byte aligned. Max size, no need to free
//#if INCLUDE_NEW_JPG
//    static struct esp_jpeg_stream jpegHandle = { 0 };
//    static uint8_t* jpgBuf = (uint8_t*)ps_malloc(RESIZE_DIM_SQ * RGB888_BYTES);
//#endif  
//
//    // calculate parameters for sample size when resolution changes
//    if (fsizePtr != fsizePtrPrev) {
//        fsizePtrPrev = fsizePtr;
//        scaling = frameData[fsizePtr].scaleFactor;
//        reducer = frameData[fsizePtr].sampleRate;
//        downsize = pow(2, scaling) * reducer;
//        stride = (colorDepth == RGB888_BYTES) ? GRAYSCALE_BYTES : RGB888_BYTES; // stride is inverse of colorDepth
//        sampleWidth = frameData[fsizePtr].frameWidth / downsize;
//        sampleHeight = frameData[fsizePtr].frameHeight / downsize;
//#if INCLUDE_NEW_JPG
//        jpg2rgbClose(&jpegHandle);
//        jpgReduce(fb->width, fb->height, downsize, &sampleWidth, &sampleHeight);
//        if (!jpg2rgbOpen(&jpegHandle, sampleWidth, sampleHeight)) return motionStatus;
//#endif
//    }
//#if INCLUDE_NEW_JPG
//    if (!jpg2rgb(&jpegHandle, fb->buf, fb->len, rgbBuf)) return motionStatus;
//#else
//    if (!jpeg2rgb((uint8_t*)fb->buf, fb->len, rgbBuf, scaling)) return motionStatus;
//#endif
//
//#if ESP_ARDUINO_VERSION >= ESP_ARDUINO_VERSION_VAL(3, 3, 0)
//    if (colorDepth == GRAYSCALE_BYTES) rgbToGray(rgbBuf, sampleWidth, sampleHeight);
//#endif
//    LOG_VRB("JPEG to rescaled %s bitmap conversion %u bytes in %lums", colorDepth == RGB888_BYTES ? "color" : "grayscale", sampleWidth * sampleHeight * colorDepth, millis() - dTime);
//
//    // allocate buffer space on heap
//    size_t resizeDimLen = RESIZE_DIM_SQ * colorDepth; // byte size of bitmap
//    if (motionJpeg == NULL) motionJpeg = (uint8_t*)ps_malloc(32 * 1024);
//    if (currBuff == NULL) currBuff = (uint8_t*)ps_malloc(RESIZE_DIM_SQ * RGB888_BYTES);
//    static uint8_t* prevBuff = (uint8_t*)ps_malloc(RESIZE_DIM_SQ * RGB888_BYTES);
//    static uint8_t* changeMap = (uint8_t*)ps_malloc(RESIZE_DIM_SQ * RGB888_BYTES);
//
//    dTime = millis();
//    rescaleImage(rgbBuf, sampleWidth, sampleHeight, currBuff, RESIZE_DIM, RESIZE_DIM);
//    LOG_VRB("Bitmap rescale to %u bytes in %lums", resizeDimLen, millis() - dTime);
//    // compare each pixel in current frame with previous frame 
//    dTime = millis();
//    int changeCount = 0;
//    // set horizontal region of interest in image 
//    uint16_t startPixel = (RESIZE_DIM * (detectStartBand - 1) / detectNumBands) * RESIZE_DIM * colorDepth;
//    uint16_t endPixel = (RESIZE_DIM * (detectEndBand) / detectNumBands) * RESIZE_DIM * colorDepth;
//    int moveThreshold = max(1, (int)(((endPixel - startPixel) / colorDepth) * (11 - motionVal) / 100)); // number of changed pixels that constitute a movement
//    for (size_t i = 0; i < resizeDimLen; i += colorDepth) {
//        uint16_t currPix = 0, prevPix = 0;
//        for (int j = 0; j < colorDepth; j++) {
//            currPix += currBuff[i + j];
//            prevPix += prevBuff[i + j];
//        }
//        currPix /= colorDepth;
//        prevPix /= colorDepth;
//        lux += currPix; // for calculating light level
//        uint8_t pixVal = 255; // show active changed pixel as bright red color in changeMap image
//        // set up display image for motion tracking debug
//        if (dbgMotion) for (int j = 0; j < RGB888_BYTES; j++) changeMap[(i * stride) + j] = currPix; // grayscale
//        // determine pixel change status
//        if (abs((int)currPix - (int)prevPix) > detectChangeThreshold) {
//            if (i > startPixel && i < endPixel) changeCount++; // number of changed pixels
//            else pixVal = 80; // show inactive changed pixel as dark red color in changeMap image
//            if (dbgMotion) {
//                changeMap[(i * stride) + 2] = pixVal;
//                for (int j = 0; j < RGB888_BYTES - 1; j++) changeMap[(i * stride) + j] = 0;
//            }
//        }
//    }
//
//    lightLevel = (lux * 100) / (RESIZE_DIM_SQ * 255); // light value as a %
//    nightTime = isNight(nightSwitch);
//    memcpy(prevBuff, currBuff, resizeDimLen); // save image for next comparison 
//    LOG_VRB("Detected %u changes, threshold %u, light level %u, in %lums", changeCount, moveThreshold, lightLevel, millis() - dTime);
//    if (lightLevelOnly) return false; // no motion checking, only calc of light level
//    if (dbgMotion) {
//        // show motion detection during streaming for tuning
//        if (!motionJpegLen) {
//            // ready to setup next movement map for streaming
//            dTime = millis();
//            // build jpeg of changeMap for debug streaming
//#if INCLUDE_NEW_JPG
//            motionJpegLen = rgb2jpg(changeMap, RESIZE_DIM, RESIZE_DIM, JPEG_QUAL, jpgBuf);
//            if (motionJpegLen == 0) LOG_WRN("motionDetect: encode() failed");
//            else memcpy(motionJpeg, jpgBuf, motionJpegLen);
//#else
//            uint8_t* jpg_buf = NULL;
//            if (!fmt2jpg(changeMap, resizeDimLen, RESIZE_DIM, RESIZE_DIM, PIXFORMAT_RGB888, JPEG_QUAL, &jpg_buf, &motionJpegLen))
//                LOG_WRN("motionDetect: fmt2jpg() failed");
//            else memcpy(motionJpeg, jpg_buf, motionJpegLen);
//            free(jpg_buf);
//            jpg_buf = NULL;
//#endif
//            if (motionJpegLen) xSemaphoreGive(motionSemaphore);
//            LOG_VRB("Created changeMap JPEG %d bytes in %lums", motionJpegLen, millis() - dTime);
//        }
//    }
//    else {
//        // normal motion detection
//        dTime = millis();
//        if (!nightTime && changeCount > moveThreshold) {
//            LOG_VRB("### Change detected");
//            motionCnt++; // number of consecutive changes
//            // need minimum sequence of changes to signal valid movement
//            if (!motionStatus && motionCnt >= detectMotionFrames) {
//                LOG_VRB("***** Motion - START");
//                motionStatus = true; // motion started
//#if INCLUDE_TINYML
//                // pass image to TinyML for classification
//                if (mlUse) if (!tinyMLclassify()) {
//                    motionCnt = 0; // not classified, so cancel motion
//                    motionStatus = false;
//                }
//#endif
//                dTime = millis();
//#if INCLUDE_MQTT
//                if (mqtt_active && motionCnt) {
//                    sprintf(jsonBuff, "{\"MOTION\":\"ON\",\"TIME\":\"%s\"}", esp_log_system_timestamp());
//                    mqttPublish(jsonBuff);
//                    mqttPublishPath("motion", "on");
//#if INCLUDE_HASIO
//                    mqttPublishPath("cmd", "still");
//#endif
//                }
//#endif
//            }
//        }
//        else motionCnt = 0;
//
//        if (motionStatus && !motionCnt) {
//            // insufficient change or motion not classified
//            LOG_VRB("***** Motion - STOP");
//            motionStatus = false; // motion stopped
//#if INCLUDE_MQTT
//            if (mqtt_active) {
//                sprintf(jsonBuff, "{\"MOTION\":\"OFF\",\"TIME\":\"%s\"}", esp_log_system_timestamp());
//                mqttPublish(jsonBuff);
//                mqttPublishPath("motion", "off");
//            }
//#endif
//        }
//        if (motionStatus) LOG_VRB("*** Motion - ongoing %lu frames", motionCnt);
//    }
//
//    if (dbgVerbose) checkMemory();
//    LOG_VRB("============================");
//    // motionStatus indicates whether motion previously ongoing or not
//    return nightTime ? false : motionStatus;
//}

// Refactored: no floating‑point, uses SAD for pixel comparison.
bool checkMotion(camera_fb_t* fb, bool motionStatus, bool lightLevelOnly) {
    // Abort if requested frame size exceeds supported maximum (SXGA)
    if (fsizePtr > FRAMESIZE_SXGA) return false;

    uint32_t dTime = millis();
    uint32_t luxSum = 0;

    // Static variables persist across calls to maintain state and avoid reallocation
    static uint32_t motionCnt = 0;
    static uint8_t  fsizePtrPrev = 255;
    static uint8_t  scaling = 0, downsize = 0;
    static uint16_t reducer = 0;
    static int      sampleWidth = 0, sampleHeight = 0;

    // Allocate aligned buffer in SPI RAM for RGB conversion to save internal memory
    static uint8_t* rgbBuf = (uint8_t*)heap_caps_aligned_calloc(
        16, 1,
        frameData[FRAMESIZE_SXGA].frameWidth *
        frameData[FRAMESIZE_SXGA].frameHeight *
        RGB888_BYTES / 8,
        MALLOC_CAP_SPIRAM);

#if INCLUDE_NEW_JPG
    static struct esp_jpeg_stream jpegHandle = { 0 };
    // Allocate buffer for JPEG processing in PSRAM
    static uint8_t* jpgBuf = (uint8_t*)ps_malloc(RESIZE_DIM_SQ * RGB888_BYTES);
#endif

    // Reconfigure sampling parameters if the camera frame size has changed
    if (fsizePtr != fsizePtrPrev) {
        fsizePtrPrev = fsizePtr;
        scaling = frameData[fsizePtr].scaleFactor;
        reducer = frameData[fsizePtr].sampleRate;
        downsize = (1u << scaling) * reducer;

        stride = (colorDepth == RGB888_BYTES) ? GRAYSCALE_BYTES : RGB888_BYTES;
        sampleWidth = frameData[fsizePtr].frameWidth / downsize;
        sampleHeight = frameData[fsizePtr].frameHeight / downsize;

#if INCLUDE_NEW_JPG
        // Reset JPEG decoder and configure new dimensions for downsizing
        jpg2rgbClose(&jpegHandle);
        jpgReduce(fb->width, fb->height, downsize, &sampleWidth, &sampleHeight);
        if (!jpg2rgbOpen(&jpegHandle, sampleWidth, sampleHeight)) {
            return motionStatus;
        }
#endif
    }

    // Decode JPEG camera buffer into raw RGB/Grayscale bitmap
#if INCLUDE_NEW_JPG
    if (!jpg2rgb(&jpegHandle, fb->buf, fb->len, rgbBuf)) return motionStatus;
#else
    if (!jpg2rgb((uint8_t*)fb->buf, fb->len, rgbBuf, scaling)) return motionStatus;
#endif

#if ESP_ARDUINO_VERSION >= ESP_ARDUINO_VERSION_VAL(3, 3, 0)
    // Convert RGB bitmap to grayscale if configured for monochrome processing
    if (colorDepth == GRAYSCALE_BYTES) rgbToGray(rgbBuf, sampleWidth, sampleHeight);
#endif

    LOG_VRB("JPEG to rescaled %s bitmap conversion %u bytes in %lums",
        colorDepth == RGB888_BYTES ? "color" : "grayscale",
        sampleWidth * sampleHeight * colorDepth,
        millis() - dTime);

    size_t resizeDimLen = RESIZE_DIM_SQ * colorDepth;

    // Lazily allocate working buffers in PSRAM if not already initialized
    if (motionJpeg == NULL) motionJpeg = (uint8_t*)ps_malloc(32 * 1024);
    if (currBuff == NULL) currBuff = (uint8_t*)ps_malloc(RESIZE_DIM_SQ * RGB888_BYTES);
    static uint8_t* prevBuff = (uint8_t*)ps_malloc(RESIZE_DIM_SQ * RGB888_BYTES);
    static uint8_t* changeMap = (uint8_t*)ps_malloc(RESIZE_DIM_SQ * RGB888_BYTES);

    dTime = millis();
    // Downscale image to fixed resolution for efficient motion comparison
    rescaleImage(rgbBuf, sampleWidth, sampleHeight,
        currBuff, RESIZE_DIM, RESIZE_DIM);
    LOG_VRB("Bitmap rescale to %u bytes in %lums", resizeDimLen, millis() - dTime);

    // Calculate pixel indices for the Region of Interest (ROI) based on band settings
    uint32_t startPixelRow = (static_cast<uint32_t>(RESIZE_DIM) * (detectStartBand - 1)) / detectNumBands;
    uint32_t endPixelRow = (static_cast<uint32_t>(RESIZE_DIM) * detectEndBand) / detectNumBands;
    uint32_t startPixelIdx = startPixelRow * static_cast<uint32_t>(RESIZE_DIM);
    uint32_t endPixelIdx = endPixelRow * static_cast<uint32_t>(RESIZE_DIM);
    uint32_t roiPixels = (endPixelIdx > startPixelIdx) ? (endPixelIdx - startPixelIdx) : 0;

    // Determine motion sensitivity threshold: higher motionVal means lower threshold
    int moveThreshold = static_cast<int>((roiPixels * (11 - motionVal)) / 100);
    if (moveThreshold < 1) moveThreshold = 1;

    int changeCount = 0;
    const uint16_t sadThreshold = static_cast<uint16_t>(detectChangeThreshold) * colorDepth;
    uint32_t totalPixels = resizeDimLen / colorDepth;

    const uint8_t* pCurr = currBuff;
    const uint8_t* pPrev = prevBuff;
    uint8_t* pMap = changeMap;

    // Iterate through every pixel to calculate difference from previous frame
    for (uint32_t pixelIdx = 0; pixelIdx < totalPixels; ++pixelIdx) {
        uint16_t channelSum = 0;
        uint16_t sad = 0;

        // Sum absolute differences (SAD) across all color channels
        for (int j = 0; j < colorDepth; ++j) {
            uint8_t cVal = *pCurr++;
            uint8_t pVal = *pPrev++;

            channelSum += cVal;

            int diff = static_cast<int>(cVal) - static_cast<int>(pVal);
            sad += static_cast<uint16_t>((diff < 0) ? -diff : diff);
        }

        // Accumulate brightness values for light level calculation
        luxSum += channelSum;

        if (dbgMotion) {
            // Fill change map with grayscale value for visual debugging
            uint8_t grayVal = static_cast<uint8_t>(channelSum / colorDepth);
            for (int j = 0; j < RGB888_BYTES; ++j) {
                *pMap++ = grayVal;
            }
        }
        else {
            // Skip writing to change map to save processing time
            pMap += RGB888_BYTES;
        }

        // Check if pixel change exceeds sensitivity threshold
        if (sad > sadThreshold) {
            // Only count changes within the defined Region of Interest
            if (pixelIdx >= startPixelIdx && pixelIdx < endPixelIdx) {
                ++changeCount;
            }
            else if (dbgMotion) {
                // Mark out-of-ROI motion as red in debug image
                uint8_t* pMarker = pMap - RGB888_BYTES;
                pMarker[0] = 0;   // B
                pMarker[1] = 0;   // G
                pMarker[2] = 255; // R (Red for motion)
            }
        }
    }

    // Normalize accumulated brightness to a 0-100% light level scale
    uint32_t maxChannelSum = static_cast<uint32_t>(RESIZE_DIM_SQ) * 255u * static_cast<uint32_t>(colorDepth);

    if (maxChannelSum > 0) {
        lightLevel = static_cast<uint32_t>((luxSum * 100u) / maxChannelSum);
    }
    else {
        lightLevel = 0;
    }

    // Update night mode status based on light sensor or calculated level
    nightTime = isNight(nightSwitch);

    // Save current frame as previous frame for next iteration's comparison
    memcpy(prevBuff, currBuff, resizeDimLen);

    LOG_VRB("Detected %u changes, threshold %d, light level %u%%, in %lums",
        changeCount, moveThreshold, lightLevel, millis() - dTime);

    // If only light level data is requested, skip motion detection logic
    if (lightLevelOnly) return false;

    if (dbgMotion) {
        // Encode change map into JPEG for remote debugging view
        if (!motionJpegLen) {
            dTime = millis();
#if INCLUDE_NEW_JPG
            motionJpegLen = rgb2jpg(changeMap, RESIZE_DIM, RESIZE_DIM,
                JPEG_QUAL, jpgBuf);
            if (motionJpegLen == 0) LOG_WRN("motionDetect: encode() failed");
            else memcpy(motionJpeg, jpgBuf, motionJpegLen);
#else
            uint8_t* jpg_buf = nullptr;
            if (!fmt2jpg(changeMap, resizeDimLen,
                RESIZE_DIM, RESIZE_DIM,
                PIXFORMAT_RGB888, JPEG_QUAL,
                &jpg_buf, &motionJpegLen))
                LOG_WRN("motionDetect: fmt2jpg() failed");
            else {
                memcpy(motionJpeg, jpg_buf, motionJpegLen);
                free(jpg_buf);
            }
#endif
            // Signal semaphore to notify task that debug image is ready
            if (motionJpegLen) xSemaphoreGive(motionSemaphore);
            LOG_VRB("Created changeMap JPEG %d bytes in %lums", motionJpegLen, millis() - dTime);
        }
    }
    else {
        dTime = millis();

        // Detect motion only if it's daytime and changes exceed threshold
        if (!nightTime && changeCount > moveThreshold) {
            LOG_VRB("### Change detected");
            ++motionCnt;

            // Confirm motion after consecutive frames meet threshold
            if (!motionStatus && motionCnt >= static_cast<uint32_t>(detectMotionFrames)) {
                LOG_VRB("***** Motion - START");
                motionStatus = true;

#if INCLUDE_TINYML
                // Optional: Validate motion using TinyML classifier
                if (mlUse && !tinyMLclassify()) {
                    motionCnt = 0;
                    motionStatus = false;
                }
#endif
                dTime = millis();

#if INCLUDE_MQTT
                // Publish motion start event via MQTT if enabled
                if (mqtt_active && motionCnt) {
                    sprintf(jsonBuff,
                        "{\"MOTION\":\"ON\",\"TIME\":\"%s\"}",
                        esp_log_system_timestamp());
                    mqttPublish(jsonBuff);
                    mqttPublishPath("motion", "on");
#if INCLUDE_HASIO
                    mqttPublishPath("cmd", "still");
#endif
                }
#endif
            }
        }
        else {
            // Reset counter if no significant change or during night mode
            motionCnt = 0;
        }

        // Clear motion status if counter resets while motion was active
        if (motionStatus && motionCnt == 0) {
            LOG_VRB("***** Motion - STOP");
            motionStatus = false;

#if INCLUDE_MQTT
            // Publish motion stop event via MQTT if enabled
            if (mqtt_active) {
                sprintf(jsonBuff,
                    "{\"MOTION\":\"OFF\",\"TIME\":\"%s\"}",
                    esp_log_system_timestamp());
                mqttPublish(jsonBuff);
                mqttPublishPath("motion", "off");
            }
#endif
        }

        if (motionStatus) LOG_VRB("*** Motion - ongoing %lu frames", motionCnt);
    }

    if (dbgVerbose) checkMemory();

    LOG_VRB("============================");
    // Return false if night mode is active, otherwise return current motion status
    return nightTime ? false : motionStatus;
}



/* ----------------------------------------------------------------
 *  Helper to build a camera framebuffer filled with a constant byte
 * ---------------------------------------------------------------- */
camera_fb_t make_fb(uint8_t fill)
{
    camera_fb_t fb;
    fb.width = RESIZE_DIM;
    fb.height = RESIZE_DIM;
    fb.len = RESIZE_DIM_SQ * RGB888_BYTES;
    fb.buf = (uint8_t*)std::malloc(fb.len);
    std::memset(fb.buf, fill, fb.len);
    return fb;
}
void free_fb(camera_fb_t& fb)
{
    std::free(fb.buf);
    fb.buf = nullptr;
}

/* ----------------------------------------------------------------
 *  Unit tests
 * ---------------------------------------------------------------- */
bool test_no_motion()
{
    camera_fb_t fb0 = make_fb(0x00);
    bool result = checkMotion(&fb0, false, false);
    free_fb(fb0);
    if (result) {
        std::cerr << "test_no_motion FAILED – expected false, got true\n";
        return false;
    }
    return true;
}
bool test_motion_detected()
{
    /* first call – blank frame to initialise the previous buffer */
    camera_fb_t fb0 = make_fb(0x00);
    checkMotion(&fb0, false, false);
    free_fb(fb0);
    /* second call – bright frame should trigger motion */
    camera_fb_t fb1 = make_fb(0xFF);
    bool result = checkMotion(&fb1, false, false);
    free_fb(fb1);
    if (!result) {
        std::cerr << "test_motion_detected FAILED – expected true, got false\n";
        return false;
    }
    return true;
}
bool test_light_level_only()
{
    camera_fb_t fb = make_fb(0x80);
    bool result = checkMotion(&fb, false, true);
    free_fb(fb);
    if (result) {
        std::cerr << "test_light_level_only FAILED – expected false, got true\n";
        return false;
    }
    return true;
}

/* ----------------------------------------------------------------
 *  Benchmark – runs `iterations` calls and prints average µs per call
 * ---------------------------------------------------------------- */
void benchmark(int iterations)
{
    camera_fb_t fb0 = make_fb(0x00);
    camera_fb_t fb1 = make_fb(0xFF);
    bool status = false;
    auto start = std::chrono::high_resolution_clock::now();
    for (int i = 0; i < iterations; ++i) {
        camera_fb_t* cur = (i & 1) ? &fb1 : &fb0;
        status = checkMotion(cur, status, false);
    }
    auto end = std::chrono::high_resolution_clock::now();
    double us = std::chrono::duration<double, std::micro>(end - start).count();
    std::cout << "Benchmark: " << iterations << " calls, average "
        << (us / iterations) << " µs per call\n";
    free_fb(fb0);
    free_fb(fb1);
}

/* ----------------------------------------------------------------
 *  Main – runs the three unit tests and then the benchmark
 * ---------------------------------------------------------------- */
int main()
{
    init_globals();

    std::cout << "Running unit tests …\n";
    int passed = 0, total = 3;
    if (test_no_motion())      ++passed;
    if (test_motion_detected())++passed;
    if (test_light_level_only())++passed;
    std::cout << passed << '/' << total << " tests passed.\n\n";

    std::cout << "Running benchmark (1000 iterations)…\n";
    benchmark(1000);
    return 0;
}
