
/* 
 Detect movement in sequential images using background subtraction.
 
 Very small (96x96) bitmaps are used both to provide image smoothing to reduce spurious motion changes 
 and to enable rapid processing
 Bitmaps can either be color or grayscale. Color requires triple memory
 of grayscale and more processing.

 The amount of change between images will depend on the frame rate.
 A faster frame rate will need a higher sensitivity

 When frame size is changed the OV2640 outputs a few glitched frames whilst it 
 makes the transition. These could be interpreted as spurious motion.

 Machine Learning can be incorporated to further discriminate when motion detection 
 has occurred by classifying whether the object in the frame is of a particular
 type of interest, eg a human, animal, vehicle etc. 
 
 s60sc 2020, 2023, 2025
*/

#include "appGlobals.h"

#if INCLUDE_TINYML
#include TINY_ML_LIB
#endif

#define RESIZE_DIM 96  // dimensions of resized motion bitmap
#define RESIZE_DIM_SQ (RESIZE_DIM * RESIZE_DIM) // pixels in bitmap
#define INACTIVE_COLOR 96 // color for inactive motion pixel
#define JPEG_QUAL 80 // % quality for generated motion detect jpeg
  
// motion recording parameters
bool dbgMotion = false;
int detectMotionFrames = 5; // min sequence of changed frames to confirm motion 
int detectNightFrames = 10; // frames of sequential darkness to avoid spurious day / night switching
// define region of interest, ie exclude top and bottom of image from movement detection if required
// divide image into detectNumBands horizontal bands, define start and end bands of interest, 1 = top
int detectNumBands = 10;
int detectStartBand = 3;
int detectEndBand = 8; // inclusive
int detectChangeThreshold = 15; // min difference in pixel comparison to indicate a change
uint8_t colorDepth; // set by depthColor config
static size_t stride;
bool mlUse = false; // whether to use ML for motion detection, requires INCLUDE_TINYML to be true
float mlProbability = 0.8; // minimum probability (0.0 - 1.0) for positive classification

uint8_t lightLevel; // Current ambient light level 
uint8_t nightSwitch = 20; // initial white level % for night/day switching
float motionVal = 8.0; // initial motion sensitivity setting
uint8_t* motionJpeg = NULL;
size_t motionJpegLen = 0;
static uint8_t* currBuff = NULL;

#ifndef AUXILIARY

#if INCLUDE_NEW_JPG
// use esp_new_jpeg library instead of built in
#include <esp_jpeg_dec.h>
#include <esp_jpeg_enc.h>

struct esp_jpeg_stream {
    jpeg_dec_handle_t       jpeg_dec;
    jpeg_dec_io_t*          jpeg_io;
    jpeg_dec_header_info_t* out_info;
    jpeg_pixel_format_t     output_type;
};
typedef struct esp_jpeg_stream* esp_jpeg_stream_handle_t;

static void jpgReduce(int inWidth, int inHeight, uint8_t downsize, int* outWidth, int* outHeight);
static bool jpg2rgbOpen(esp_jpeg_stream_handle_t jpegHandle, uint16_t width, uint16_t height);
static bool jpg2rgb(esp_jpeg_stream_handle_t jpegHandle, uint8_t* inputBuf, int inputLen, uint8_t* outputBuf);
static bool jpg2rgbClose(esp_jpeg_stream_handle_t jpegHandle);
static size_t rgb2jpg(uint8_t* rgb888, int width, int height, int qual, uint8_t* outputBuf);
#else
// built in
static bool jpg2rgb(const uint8_t* src, size_t src_len, uint8_t* out, uint8_t scale);
#endif

/**********************************************************************************/


bool isNight(uint8_t nightSwitch) {
  // check if night time for suspending recording
  // or for switching relay if enabled
  static bool nightTime = false;
  static uint16_t nightCnt = 0;
  if (nightTime) {
    if (lightLevel > nightSwitch) {
      // light image
      if (nightCnt > 0) nightCnt--;
      // signal day time after given sequence of light frames
      if (nightCnt == 0) {
        nightTime = false;
        LOG_INF("Day time");
      }
    }
  } else {
    if (lightLevel < nightSwitch) {
      // dark image
      nightCnt++;
      // signal night time after given sequence of dark frames
      if (nightCnt > detectNightFrames) {
        nightTime = true;     
        nightCnt = detectNightFrames;           
        LOG_INF("Night time"); 
      }
    } else {
      // back to light while not yet in nightTime: reset counter
      if (nightCnt > 0) nightCnt--;
    }
  } 
  return nightTime;
}

static void rescaleImage(const uint8_t* __restrict input,
    int32_t inputWidth,
    int32_t inputHeight,
    uint8_t* __restrict output,
    int32_t outputWidth,
    int32_t outputHeight)
{
    // Validate inputs and dimensions to prevent undefined behavior or invalid memory access.
    if (!input || !output || input == output ||
        inputWidth <= 0 || inputHeight <= 0 ||
        outputWidth <= 0 || outputHeight <= 0 ||
        colorDepth <= 0 || colorDepth > 4) {
        return;
    }

    // Enforce 16-bit addressing limits to ensure compatibility with embedded hardware constraints.
    if (inputWidth > 65535 || inputHeight > 65535 ||
        outputWidth > 65535 || outputHeight > 65535) {
        return;
    }

    // Skip processing if source and destination dimensions match by copying data directly.
    if (inputWidth == outputWidth && inputHeight == outputHeight) {
        size_t totalBytes = static_cast<size_t>(inputWidth) * inputHeight * colorDepth;
        memcpy(output, input, totalBytes);
        return;
    }

    // Define fixed-point constants for 8-bit fractional precision and rounding.
    enum : uint32_t {
        SHIFT8 = 8,
        WEIGHT_MASK = 0xFF,
        ROUNDING_OFFSET = 0x8000
    };

    // Calculate horizontal and vertical scaling factors in 8.8 fixed-point format.
    uint32_t xRatio = (static_cast<uint32_t>(inputWidth) << SHIFT8) / static_cast<uint32_t>(outputWidth);
    uint32_t yRatio = (static_cast<uint32_t>(inputHeight) << SHIFT8) / static_cast<uint32_t>(outputHeight);

    // Prevent division-by-zero errors by ensuring scaling ratios are at least 1.
    if (xRatio == 0) xRatio = 1;
    if (yRatio == 0) yRatio = 1;

    // Compute row strides as 16-bit values to minimize register pressure on embedded targets.
    const uint16_t inputStride = static_cast<uint16_t>(inputWidth) * static_cast<uint16_t>(colorDepth);
    const uint16_t outputStride = static_cast<uint16_t>(outputWidth) * static_cast<uint16_t>(colorDepth);

    // Determine maximum accumulator values to clamp coordinates within image boundaries.
    const uint32_t yMaxAcc = (static_cast<uint32_t>(inputHeight) - 1) << SHIFT8;
    const uint32_t xMaxAcc = (static_cast<uint32_t>(inputWidth) - 1) << SHIFT8;

    uint32_t yAcc = 0;
    uint8_t* outBase = output;

    // Iterate through output rows using a fixed-point accumulator to drive interpolation steps.
    while (true) {
        // Clamp vertical accumulator to prevent reading past the last valid input row.
        if (yAcc > yMaxAcc) yAcc = yMaxAcc;

        // Extract integer indices and fractional weights from the vertical accumulator.
        const uint16_t yL = static_cast<uint16_t>(yAcc >> SHIFT8);
        const uint16_t yH = (yL + 1 < static_cast<uint16_t>(inputHeight)) ? (yL + 1) : yL;
        const uint8_t  yFrac = static_cast<uint8_t>(yAcc & WEIGHT_MASK);

        // Calculate base pointers for the top and bottom source rows involved in this output line.
        const uint8_t* rowL = input + (static_cast<size_t>(yL) * inputStride);
        const uint8_t* rowH = input + (static_cast<size_t>(yH) * inputStride);
        uint8_t* outRow = outBase;

        uint32_t xAcc = 0;

        // Iterate through output columns using a horizontal fixed-point accumulator.
        while (true) {
            // Clamp horizontal accumulator to prevent reading past the last valid input column.
            if (xAcc > xMaxAcc) xAcc = xMaxAcc;

            // Extract integer indices and fractional weights from the horizontal accumulator.
            const uint16_t xL = static_cast<uint16_t>(xAcc >> SHIFT8);
            const uint16_t xH = (xL + 1 < static_cast<uint16_t>(inputWidth)) ? (xL + 1) : xL;
            const uint8_t  xFrac = static_cast<uint8_t>(xAcc & WEIGHT_MASK);

            // Resolve pointers to the four corners of the bilinear sampling neighborhood.
            const uint8_t* pTL = rowL + (static_cast<size_t>(xL) * colorDepth);
            const uint8_t* pTR = rowL + (static_cast<size_t>(xH) * colorDepth);
            const uint8_t* pBL = rowH + (static_cast<size_t>(xL) * colorDepth);
            const uint8_t* pBR = rowH + (static_cast<size_t>(xH) * colorDepth);

            // Process each color channel independently using bilinear interpolation logic.
            int k = 0;
            while (k < colorDepth) {
                // Load pixel values from the four corners of the sampling grid.
                const uint8_t a = pTL[k];
                const uint8_t b = pTR[k];
                const uint8_t c = pBL[k];
                const uint8_t d = pBR[k];

                // Perform linear interpolation along the horizontal axis for both top and bottom pairs.
                int32_t val_top = (static_cast<int32_t>(a) << SHIFT8) +
                    (static_cast<int32_t>(b - a) * static_cast<int32_t>(xFrac));
                int32_t val_bot = (static_cast<int32_t>(c) << SHIFT8) +
                    (static_cast<int32_t>(d - c) * static_cast<int32_t>(xFrac));

                // Perform linear interpolation along the vertical axis between the intermediate results.
                int32_t total = (val_top << SHIFT8) +
                    (static_cast<int32_t>(val_bot - val_top) * static_cast<int32_t>(yFrac));

                // Apply rounding bias, shift down, and store the final 8-bit pixel value.
                *outRow++ = static_cast<uint8_t>((total + ROUNDING_OFFSET) >> 16);

                k++;
            }

            // Increment the horizontal accumulator to advance to the next output column.
            xAcc += xRatio;

            // Terminate loop once the accumulator exceeds the maximum valid coordinate bound.
            if (xAcc > xMaxAcc) break;
        }

        // Increment the vertical accumulator and advance the output buffer pointer to the next row.
        yAcc += yRatio;
        outBase += outputStride;

        // Terminate loop once the accumulator exceeds the maximum valid row coordinate bound.
        if (yAcc > yMaxAcc) break;
    }
}

static inline void rgbToGray(uint8_t* buffer, int width, int height) {
    // Guard against invalid inputs to prevent undefined behavior.
    if (!buffer || width <= 0 || height <= 0) return;

    int totalPixels = width * height;
    const uint8_t* src = buffer;
    uint8_t* dst = buffer;

    // Fixed-point coefficients scaled by 256 for integer-only arithmetic.
    enum : uint32_t {
        COEF_R = 77,
        COEF_G = 150,
        COEF_B = 29
    };

    // Decrementing while-loop eliminates loop counter overhead.
    while (totalPixels--) {
        // Load RGB components and advance source pointer simultaneously.
        uint8_t r = *src++;
        uint8_t g = *src++;
        uint8_t b = *src++;

        // Accumulate weighted sum
        uint32_t gray_val = (COEF_R * r + COEF_G * g + COEF_B * b);

        // Normalize fixed-point result and store grayscale value.
        *dst++ = (uint8_t)(gray_val >> 8);
    }
}

#if INCLUDE_TINYML

static int getImageData(size_t offset, size_t length, float *out_ptr) {
  // copy to features as grayscale or RGB
  size_t pixelPtr = offset * colorDepth;
  size_t out_ptr_idx = 0;
  while (out_ptr_idx < length) {
    out_ptr[out_ptr_idx++] = (colorDepth == RGB888_BYTES)  
      ? (float)((currBuff[pixelPtr] << 16) + (currBuff[pixelPtr + 1] << 8) + currBuff[pixelPtr + 2])
      : (float)((currBuff[pixelPtr] << 16) + (currBuff[pixelPtr] << 8) + currBuff[pixelPtr]);  
    pixelPtr += colorDepth;
  } 
  return 0;
}

static bool tinyMLclassify() {
  // convert input data to appropriate format
  bool out = false;
  uint32_t dTime = millis(); 
  // reduce size of bitmap to that required by classifier and copy to features as grayscale or RGB
  if (RESIZE_DIM != EI_CLASSIFIER_INPUT_WIDTH) {
    size_t tempSize = EI_CLASSIFIER_INPUT_WIDTH * EI_CLASSIFIER_INPUT_HEIGHT * colorDepth;
    uint8_t* tempBuff = (uint8_t*)ps_malloc(tempSize);
    rescaleImage(currBuff, RESIZE_DIM, RESIZE_DIM, tempBuff, EI_CLASSIFIER_INPUT_WIDTH, EI_CLASSIFIER_INPUT_HEIGHT);
    memcpy(currBuff, tempBuff, tempSize);
    free(tempBuff);
  }
  signal_t features_signal;
  features_signal.total_length = EI_CLASSIFIER_INPUT_WIDTH * EI_CLASSIFIER_INPUT_HEIGHT;
  features_signal.get_data = &getImageData;

  // Run the classifier
  ei_impulse_result_t result = { 0 };
  EI_IMPULSE_ERROR res = run_classifier(&features_signal, &result, false);
  if (res == EI_IMPULSE_OK) {
    if (result.classification[0].value > mlProbability) {
      out = true; // sufficient classification match, so keep motion detection
      if (dbgVerbose) {
        LOG_VRB("Prob: %0.2f, Timing: DSP %d ms, inference %d ms, anomaly %d ms", 
        result.classification[0].value, result.timing.dsp, result.timing.classification, result.timing.anomaly);
        char outcome[200] = {0};
        for (uint16_t i = 0; i < EI_CLASSIFIER_LABEL_COUNT; i++)
          sprintf(outcome + strlen(outcome), "%s: %.2f, ", ei_classifier_inferencing_categories[i], result.classification[i].value);
        LOG_VRB("Predictions - %s in %ums", outcome, millis() - dTime);
      } 
    } 
  } else LOG_WRN("Failed to run classifier (%d)", res);
  return out;
}
#endif

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

/*****************************************************************************************************/

#if INCLUDE_NEW_JPG

// Need to have installed espressif__esp_new_jpeg library

static void jpgReduce(int inWidth, int inHeight, uint8_t downsize, int* outWidth, int* outHeight) {
  // downsize then round width and height up to the nearest multiple of 8 while preserving the aspect ratio
  uint8_t roundTo8 = 8; // new width and height must be multiples of 8
  // Calculate the original aspect ratio 
  inWidth /= downsize;
  inHeight /= downsize;
  float aspectRatio = (float)(inWidth) / inHeight;

  auto roundUpToMultiple = [](int n, int m) {
    // round n up to the nearest multiple of m
    return ((n + m - 1) / m) * m;
  };

  // determine larger dimension
  int newLarger = inWidth;
  int newSmaller = inHeight;   
  if (inWidth < inHeight) {
    newLarger = inHeight;
    newSmaller = inWidth;
  }

  // Round the larger dimension up to the nearest multiple of 8.
  newLarger = roundUpToMultiple(inWidth, roundTo8);
  
  // Calculate the new smaller based on the new larger and original aspect ratio, then round up.
  newSmaller = (int)(ceil((float)newLarger / aspectRatio));
  newSmaller = roundUpToMultiple(newSmaller, roundTo8);

  // update the values to return
  *outWidth = newLarger;
  *outHeight = newSmaller;
  if (inWidth < inHeight) {
    *outWidth = newSmaller;
    *outHeight = newLarger;
  }
}

static bool jpg2rgbOpen(esp_jpeg_stream_handle_t jpegHandle, uint16_t width, uint16_t height) {
  // configure jpeg handler
  jpeg_dec_config_t config = DEFAULT_JPEG_DEC_CONFIG();
  config.output_type = JPEG_PIXEL_FORMAT_RGB888;
  config.rotate = JPEG_ROTATE_0D;
  config.scale.width = width;
  config.scale.height = height;
  jpegHandle->output_type = JPEG_PIXEL_FORMAT_RGB888;

  // Create jpeg_dec handle
  jpeg_error_t ret = jpeg_dec_open(&config, &jpegHandle->jpeg_dec);
  if (ret != JPEG_ERR_OK) {
    LOG_ERR("Unable to create jpeg decoder handle: %d", ret);
    return false;
  }

  // Create io_callback handle
  jpegHandle->jpeg_io = (jpeg_dec_io_t*)calloc(1, sizeof(jpeg_dec_io_t));
  if (jpegHandle->jpeg_io == NULL) {
    LOG_ERR("Insufficient memory to create input handle");
    jpg2rgbClose(jpegHandle);
    return false;
  }

  // Create out_info handle
  jpegHandle->out_info = (jpeg_dec_header_info_t*)calloc(1, sizeof(jpeg_dec_header_info_t));
  if (jpegHandle->out_info == NULL) {
    LOG_ERR("Insufficient memory to create output handle");
    jpg2rgbClose(jpegHandle);
    return false;
  }
  return true;
}

static bool jpg2rgb(esp_jpeg_stream_handle_t jpegHandle, uint8_t* inputBuf, int inputLen, uint8_t* outputBuf) {
  // decode jpeg to rgb888
  // Set input buffer and buffer len to io_callback
  jpegHandle->jpeg_io->inbuf = inputBuf;
  jpegHandle->jpeg_io->inbuf_len = inputLen;

  // Parse jpeg header and get image for decoder
  jpeg_error_t ret = jpeg_dec_parse_header(jpegHandle->jpeg_dec, jpegHandle->jpeg_io, jpegHandle->out_info);
  if (ret != JPEG_ERR_OK) {
    LOG_ERR("Failed to parse jpeg header: %d", ret);
    return false;
  }

  // decode jpeg into outputBuf
  jpegHandle->jpeg_io->outbuf = outputBuf;
  ret = jpeg_dec_process(jpegHandle->jpeg_dec, jpegHandle->jpeg_io);
  if (ret != JPEG_ERR_OK) {
    LOG_ERR("Failed to decode jpeg: %d", ret);
    return false;
  }
  return true;
}

static bool jpg2rgbClose(esp_jpeg_stream_handle_t jpegHandle) {
   // remove old stream handles when resolution changes
  jpeg_error_t ret = jpeg_dec_close(jpegHandle->jpeg_dec);
  if (jpegHandle->jpeg_io) free(jpegHandle->jpeg_io);
  if (jpegHandle->out_info) free(jpegHandle->out_info);
  return ret == JPEG_ERR_OK;
}

static size_t rgb2jpg(uint8_t* rgb888, int width, int height, int qual, uint8_t* outputBuf) {
  // encode rgb888 to jpeg
  static bool firstCall = true;
  static jpeg_enc_handle_t jpeg_enc = NULL;
  static int bufLen = width * height * RGB888_BYTES;
  jpeg_error_t ret = JPEG_ERR_OK;

  if (firstCall) {
    firstCall = false;
    // configure encoder
    jpeg_enc_config_t jpeg_enc_cfg = DEFAULT_JPEG_ENC_CONFIG();
    jpeg_enc_cfg.width = width;
    jpeg_enc_cfg.height = height;
    jpeg_enc_cfg.src_type = JPEG_PIXEL_FORMAT_RGB888;
    jpeg_enc_cfg.subsampling = JPEG_SUBSAMPLE_420;
    jpeg_enc_cfg.quality = qual;
    jpeg_enc_cfg.rotate = JPEG_ROTATE_0D;
    jpeg_enc_cfg.task_enable = false;
    jpeg_enc_cfg.hfm_task_priority = 13;
    jpeg_enc_cfg.hfm_task_core = 1;

    // open encoder
    ret = jpeg_enc_open(&jpeg_enc_cfg, &jpeg_enc);
    if (ret != JPEG_ERR_OK) {
      LOG_ERR("Failed to open decoder: %d", ret);
      return 0;
    }
  }

  // encoding
  int jpgLen = 0;
  ret = jpeg_enc_process(jpeg_enc, rgb888, bufLen, outputBuf, bufLen, &jpgLen);
  if (ret != JPEG_ERR_OK) LOG_ERR("Failed to encode: %d", ret);

  //jpeg_enc_close(jpeg_enc); // keep open
  return (size_t)jpgLen;
}

#else

#if ESP_ARDUINO_VERSION >= ESP_ARDUINO_VERSION_VAL(3, 3, 0)

// based on jpg2rgb888() from esp32-camera/to_bmp.c for access to rescaling

static uint8_t work[3100]; // Default size is 3.1kB for JPEG decoder

static bool jpg2rgb(const uint8_t* src, size_t src_len, uint8_t* out, uint8_t scale) {
  esp_jpeg_image_cfg_t jpeg_cfg = {
      .indata = (uint8_t *)src,
      .indata_size = src_len,
      .outbuf = out,
      .outbuf_size = UINT32_MAX, // sic @todo: this is very bold assumption, keeping this like this for now, not to break existing code
      .out_format = JPEG_IMAGE_FORMAT_RGB888,
      .out_scale = (esp_jpeg_image_scale_t)scale,
      .flags = {.swap_color_bytes = 0},
      .advanced = {
        .working_buffer = work,
        .working_buffer_size = sizeof(work)
      }
  };
  esp_jpeg_image_output_t output_img = {};
  esp_err_t res = esp_jpeg_decode(&jpeg_cfg, &output_img);
  if (res != ESP_OK) LOG_WRN("jpg2rgb failure: %s", espErrMsg(res)); 
  return (res == ESP_OK);
}

#else

// for arduino-esp32 versions 3.2.1 or earlier

/************* copied and modified from esp32-camera/to_bmp.c to access jpg_scale_t *****************/

typedef struct {
  uint16_t width;
  uint16_t height;
  uint16_t data_offset;
  const uint8_t *input;
  uint8_t *output;
} rgb_jpg_decoder;

// copy_color_row: Converts BGR source data to RGB destination using a decrementing while loop.
static inline void copy_color_row(const uint8_t* __restrict src,
    uint8_t* __restrict dst,
    uint16_t pixel_cnt) noexcept
{
    // Decrementing while loop avoids index multiplication and relies on single zero-test.
    while (pixel_cnt--) {
        // Swap BGR to RGB by reversing byte order.
        dst[0] = src[2];
        dst[1] = src[1];
        dst[2] = src[0];

        // Advance pointers by one pixel (3 bytes).
        src += RGB888_BYTES;
        dst += RGB888_BYTES;
    }
}

// copy_gray_row: Converts RGB source data to grayscale average using fixed-point arithmetic.
static inline void copy_gray_row(const uint8_t* __restrict src,
    uint8_t* __restrict dst,
    uint16_t pixel_cnt) noexcept
{
    // Fixed-point magic constant for efficient division by 3: (sum * 0x5556) >> 16.
    constexpr uint32_t DIV3_MUL = 0x5556u;

    // Decrementing while loop minimizes instruction overhead per pixel.
    while (pixel_cnt--) {
        // Accumulate RGB channels; uint16_t safely holds max sum of 765.
        uint16_t sum = static_cast<uint16_t>(src[0]) +
            static_cast<uint16_t>(src[1]) +
            static_cast<uint16_t>(src[2]);

        // Compute grayscale average via fixed-point multiplication and shift.
        *dst++ = static_cast<uint8_t>((static_cast<uint32_t>(sum) * DIV3_MUL) >> 16u);

        // Advance source pointer by one pixel (3 bytes).
        src += RGB888_BYTES;
    }
}

// _rgb_write: Optimized tile writer with branch hoisting and reduced integer types.
static bool _rgb_write(void* __restrict arg,
    uint16_t x, uint16_t y,
    uint16_t w, uint16_t h,
    const uint8_t* __restrict data) noexcept
{
    // Validate decoder context pointer.
    auto* jpeg = static_cast<rgb_jpg_decoder*>(arg);
    if (!jpeg) {
        return false;
    }

    // Store image dimensions on initial metadata call (null data).
    if (!data) {
        if (x == 0 && y == 0) {
            jpeg->width = w;
            jpeg->height = h;
        }
        return true;
    }

    // Ensure write region bounds do not exceed allocated buffer dimensions.
    if (static_cast<uint32_t>(x) + w > jpeg->width ||
        static_cast<uint32_t>(y) + h > jpeg->height) {
        return false;
    }

    // Determine output format: use minimal types to reduce register pressure.
    constexpr uint8_t srcBpp = RGB888_BYTES;
    const uint8_t dstBpp = static_cast<uint8_t>(colorDepth);
    const bool isColor = (dstBpp == srcBpp);

    // Calculate row strides in bytes; uint16_t is sufficient for VGA/SVGA resolutions.
    const uint16_t srcLineBytes = w * srcBpp;
    const uint16_t dstLineStride = jpeg->width * RGB888_BYTES;

    // Compute base destination address and tile offset using 32-bit math to prevent overflow.
    uint8_t* const dstBase = jpeg->output + jpeg->data_offset;
    const uint32_t dstRowStartOffset = static_cast<uint32_t>(y) * dstLineStride +
        static_cast<uint32_t>(x) * RGB888_BYTES;

    uint8_t* dstRowPtr = dstBase + dstRowStartOffset;
    const uint8_t* srcRowPtr = data;

    // Hoist color/gray branch outside loop to eliminate per-row conditional checks.
    if (isColor) {
        // Process color rows: convert BGR to RGB.
        while (h--) {
            copy_color_row(srcRowPtr, dstRowPtr, w);
            srcRowPtr += srcLineBytes;
            dstRowPtr += dstLineStride;
        }
    }
    else {
        // Process grayscale rows: convert RGB to average intensity.
        while (h--) {
            copy_gray_row(srcRowPtr, dstRowPtr, w);
            srcRowPtr += srcLineBytes;
            dstRowPtr += dstLineStride;
        }
    }

    return true;
}

static unsigned int _jpg_read(void * arg, size_t index, uint8_t *buf, size_t len) {
  rgb_jpg_decoder * jpeg = (rgb_jpg_decoder *)arg;
  if (buf) memcpy(buf, jpeg->input + index, len);
  return len;
}

static bool jpg2rgb(const uint8_t* src, size_t src_len, uint8_t* out, uint8_t scale) {
  rgb_jpg_decoder jpeg;
  jpeg.width = 0;
  jpeg.height = 0;
  jpeg.input = src;
  jpeg.output = out;
  jpeg.data_offset = 0;
  esp_err_t res = esp_jpg_decode(src_len, (jpg_scale_t)scale, _jpg_read, _rgb_write, (void*)&jpeg);
  if (res != ESP_OK) LOG_WRN("jpg2rgb failure: %s", espErrMsg(res)); 
  return (res == ESP_OK);
}

#endif // ESP_ARDUINO_VERSION

#endif // INCLUDE_NEW_JPG

#else 
// dummies
bool isNight(uint8_t nightSwitch) {return false;}

#endif // AUXILIARY

