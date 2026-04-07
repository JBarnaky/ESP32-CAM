#include <cstdint>
#include <cstddef>
#include <cmath>
#include <vector>
#include <iostream>
#include <chrono>

//-------------------------------------------------------------------
// Global colour depth – the function reads this value directly
//-------------------------------------------------------------------
uint8_t colorDepth = 3;

//-------------------------------------------------------------------
// Original implementation (unchanged)
//-------------------------------------------------------------------
//static void rescaleImage(const uint8_t* input, int inputWidth, int inputHeight, uint8_t* output, int outputWidth, int outputHeight) {
//    // use bilinear interpolation to resize image
//    float xRatio = (float)inputWidth / (float)outputWidth;
//    float yRatio = (float)inputHeight / (float)outputHeight;
//
//    for (int i = 0; i < outputHeight; ++i) {
//        for (int j = 0; j < outputWidth; ++j) {
//            int xL = (int)floor(xRatio * j);
//            int yL = (int)floor(yRatio * i);
//            int xH = (int)ceil(xRatio * j);
//            int yH = (int)ceil(yRatio * i);
//            float xWeight = xRatio * j - xL;
//            float yWeight = yRatio * i - yL;
//            for (int channel = 0; channel < colorDepth; ++channel) {
//                uint8_t a = input[(yL * inputWidth + xL) * colorDepth + channel];
//                uint8_t b = input[(yL * inputWidth + xH) * colorDepth + channel];
//                uint8_t c = input[(yH * inputWidth + xL) * colorDepth + channel];
//                uint8_t d = input[(yH * inputWidth + xH) * colorDepth + channel];
//
//                float pixel = a * (1 - xWeight) * (1 - yWeight) + b * xWeight * (1 - yWeight)
//                    + c * yWeight * (1 - xWeight) + d * xWeight * yWeight;
//                output[(i * outputWidth + j) * colorDepth + channel] = (uint8_t)pixel;
//            }
//        }
//    }
//}

// Q8.8 FP Refactor rescaleImage
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

//===================================================================
// Tiny test framework (no external deps)
//===================================================================
static int g_totalTests = 0;
static int g_passedTests = 0;

#define CHECK_EQUAL(actual, expected)                                         \
    do {                                                                       \
        ++g_totalTests;                                                        \
        if ((actual) != (expected)) {                                         \
            std::cerr << "FAIL: " << __FUNCTION__ << " (" << __LINE__ << "): " \
                      << #actual << " = " << static_cast<int>(actual)          \
                      << ", expected " << static_cast<int>(expected) << "\n"; \
        } else {                                                               \
            ++g_passedTests;                                                   \
        }                                                                      \
    } while (0)

static void compareBuffers(const uint8_t* got,
    const uint8_t* expect,
    size_t byteCount)
{
    for (size_t i = 0; i < byteCount; ++i) {
        CHECK_EQUAL(got[i], expect[i]);
    }
}

//===================================================================
// Unit‑tests
//===================================================================

// 1. Identity mapping – 1 channel (grayscale)
static void testIdentityGrayscale()
{
    const int w = 3, h = 2;
    colorDepth = 1;

    std::vector<uint8_t> input(w * h * colorDepth);
    for (int i = 0; i < w * h; ++i)
        input[i] = static_cast<uint8_t>(i * 10);   // pattern 0,10,20...

    std::vector<uint8_t> output(w * h * colorDepth, 0);

    rescaleImage(input.data(), w, h, output.data(), w, h);

    compareBuffers(output.data(), input.data(), input.size());
}

// 2. Identity mapping – 3 channels (RGB)
static void testIdentityRGB()
{
    const int w = 2, h = 2;
    colorDepth = 3;

    std::vector<uint8_t> input(w * h * colorDepth);
    for (int y = 0; y < h; ++y)
        for (int x = 0; x < w; ++x)
            for (int c = 0; c < colorDepth; ++c) {
                int pixelIdx = y * w + x;
                input[(pixelIdx * colorDepth) + c] =
                    static_cast<uint8_t>(pixelIdx * colorDepth + c);
            }

    std::vector<uint8_t> output(w * h * colorDepth, 0);

    rescaleImage(input.data(), w, h, output.data(), w, h);

    compareBuffers(output.data(), input.data(), input.size());
}

// 3. Down‑scale 4×4 → 2×2 (grayscale, integer factor 2)
static void testDownscaleGrayscale()
{
    const int srcW = 4, srcH = 4;
    const int dstW = 2, dstH = 2;
    colorDepth = 1;

    std::vector<uint8_t> src(srcW * srcH * colorDepth);
    for (int y = 0; y < srcH; ++y)
        for (int x = 0; x < srcW; ++x)
            src[(y * srcW + x) * colorDepth] = static_cast<uint8_t>(y * srcW + x);

    std::vector<uint8_t> dst(dstW * dstH * colorDepth, 0);

    rescaleImage(src.data(), srcW, srcH, dst.data(), dstW, dstH);

    // Expected: (0,0)->0   (0,1)->2   (1,0)->8   (1,1)->10
    std::vector<uint8_t> expected = {
        0, 2,
        8,10
    };
    compareBuffers(dst.data(), expected.data(), expected.size());
}

// 4. Down‑scale 4×4 → 2×2 (RGB, integer factor 2)
static void testDownscaleRGB()
{
    const int srcW = 4, srcH = 4;
    const int dstW = 2, dstH = 2;
    colorDepth = 3;

    const size_t srcSize = srcW * srcH * colorDepth;
    std::vector<uint8_t> src(srcSize);

    // Fill with a predictable pattern:
    //   pixel index = y*srcW + x
    //   channel 0 = pixelIdx*3
    //   channel 1 = pixelIdx*3+1
    //   channel 2 = pixelIdx*3+2
    for (int y = 0; y < srcH; ++y) {
        for (int x = 0; x < srcW; ++x) {
            int pixelIdx = y * srcW + x;
            for (int c = 0; c < colorDepth; ++c) {
                src[(pixelIdx * colorDepth) + c] =
                    static_cast<uint8_t>(pixelIdx * colorDepth + c);
            }
        }
    }

    std::vector<uint8_t> dst(dstW * dstH * colorDepth, 0);
    rescaleImage(src.data(), srcW, srcH, dst.data(), dstW, dstH);

    // Verify that each destination pixel copies the source pixel at
    // (x*2, y*2) because the scaling factor is exactly 2.
    for (int dy = 0; dy < dstH; ++dy) {
        for (int dx = 0; dx < dstW; ++dx) {
            int sx = dx * 2;
            int sy = dy * 2;
            int srcPixelIdx = sy * srcW + sx;
            for (int c = 0; c < colorDepth; ++c) {
                uint8_t expected = src[(srcPixelIdx * colorDepth) + c];
                uint8_t actual = dst[(dy * dstW + dx) * colorDepth + c];
                CHECK_EQUAL(actual, expected);
            }
        }
    }
}

//===================================================================
// Simple benchmark (1920×1080 → 1280×720, 10 runs)
//===================================================================
static void benchmarkResize()
{
    const int srcW = 1920, srcH = 1080;
    const int dstW = 1280, dstH = 720;
    colorDepth = 3;                     // RGB

    const size_t srcSize = srcW * srcH * colorDepth;
    const size_t dstSize = dstW * dstH * colorDepth;

    std::vector<uint8_t> src(srcSize);
    std::vector<uint8_t> dst(dstSize, 0);

    // Fill the source with a repeating 0‑255 pattern – cheap and
    // deterministic.
    for (size_t i = 0; i < srcSize; ++i) {
        src[i] = static_cast<uint8_t>(i & 0xFF);
    }

    const int iterations = 100;
    using Clock = std::chrono::high_resolution_clock;

    auto t0 = Clock::now();
    for (int i = 0; i < iterations; ++i) {
        rescaleImage(src.data(), srcW, srcH, dst.data(), dstW, dstH);
    }
    auto t1 = Clock::now();

    std::chrono::duration<double, std::milli> elapsed = t1 - t0;
    double avgMs = elapsed.count() / iterations;

    std::cout << "\n=== Benchmark ===\n";
    std::cout << "Resize " << srcW << "*" << srcH
        << " -> " << dstW << "*" << dstH
        << " (" << srcSize << " -> " << dstSize << " bytes)\n";
    std::cout << "Iterations: " << iterations << "\n";
    std::cout << "Total time:  " << elapsed.count() << " ms\n";
    std::cout << "Average per run: " << avgMs << " ms\n";
}

//===================================================================
// Program entry point
//===================================================================
int main()
{
    std::cout << "Running unit tests...\n";

    testIdentityGrayscale();
    testIdentityRGB();
    testDownscaleGrayscale();
    testDownscaleRGB();

    std::cout << "\nTests passed: " << g_passedTests << " / " << g_totalTests << "\n";
    if (g_passedTests != g_totalTests) {
        std::cerr << "Some tests FAILED – aborting.\n";
        return 1;
    }

    benchmarkResize();

    return 0;
}
