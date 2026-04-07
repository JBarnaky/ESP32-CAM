#include <iostream>
#include <cstdint>
#include <cstddef>
#include <chrono>
#include <vector>
#include <stdexcept>
#include <cstring>      // std::memset, std::memcmp
#include <cassert>

 /*=====================================================================
  *  Global configuration – the same symbols that the original function
  *  expects to see.
  *====================================================================*/

static const int RGB888_BYTES = 3;   // 3 bytes per pixel in true‑color mode
static int stride = 1;              // 1 byte stride → “no padding”
static int colorDepth = RGB888_BYTES; // 3 → RGB, any other value → 8‑bit grayscale

/*=====================================================================
 *  Minimal decoder context (only the fields accessed by _rgb_write).
 *====================================================================*/

struct rgb_jpg_decoder {
    uint16_t width = 0;        // image width in pixels (set on the first call)
    uint16_t height = 0;        // image height in pixels (set on the first call)
    uint8_t* output = nullptr; // destination buffer (allocated by the caller)
    size_t   data_offset = 0;   // byte offset from the beginning of output
};

/*=====================================================================
 *  Function under test – copied verbatim from the question.
 *====================================================================*/

//static bool _rgb_write(void* arg, uint16_t x, uint16_t y, uint16_t w, uint16_t h, uint8_t* data) {
//    // mpjpeg2sd: modified to generate 24 bit RGB or 8 bit grayscale
//    rgb_jpg_decoder* jpeg = (rgb_jpg_decoder*)arg;
//    if (!data) {
//        if (x == 0 && y == 0) {
//            // write start
//            jpeg->width = w;
//            jpeg->height = h;
//        }
//        return true;
//    }
//
//    size_t jw = jpeg->width * RGB888_BYTES;
//    size_t t = y * jw;
//    size_t b = t + (h * jw);
//    size_t l = x * RGB888_BYTES;
//    uint8_t* out = jpeg->output + jpeg->data_offset;
//    uint8_t* o = out;
//    size_t iy, ix;
//    w *= RGB888_BYTES;
//
//    for (iy = t; iy < b; iy += jw) {
//        o = out + (iy + l) / stride;
//        for (ix = 0; ix < w; ix += RGB888_BYTES) {
//            if (colorDepth == RGB888_BYTES) {
//                o[ix] = data[ix + 2];
//                o[ix + 1] = data[ix + 1];
//                o[ix + 2] = data[ix];
//            }
//            else {
//                uint16_t grayscale = (data[ix + 2] + data[ix + 1] + data[ix]) / RGB888_BYTES;
//                o[ix / RGB888_BYTES] = (uint8_t)grayscale;
//            }
//        }
//        data += w;
//    }
//    return true;
//}

// Refactor _rgb_write
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



/*=====================================================================
 *  Tiny test‑framework helpers
 *====================================================================*/

static int g_totalTests = 0;
static int g_passedTests = 0;

[[noreturn]] static void test_failure(const char* msg) {
    throw std::runtime_error(msg);
}
static void assert_true(bool cond, const char* msg) {
    if (!cond) test_failure(msg);
}

/*--------------------------------------------------------------
 *  1. Initialise the decoder (first call with data == nullptr)
 *--------------------------------------------------------------*/
static void test_init()
{
    rgb_jpg_decoder jpeg{};
    jpeg.output = nullptr; // not needed for this test

    const uint16_t imgW = 123, imgH = 45;
    bool ok = _rgb_write(&jpeg, 0, 0, imgW, imgH, nullptr);
    assert_true(ok, "Init call must return true");
    assert_true(jpeg.width == imgW, "Width not stored correctly on init");
    assert_true(jpeg.height == imgH, "Height not stored correctly on init");

    // Call with non‑zero coordinates – dimensions must stay unchanged
    ok = _rgb_write(&jpeg, 1, 1, 10, 10, nullptr);
    assert_true(ok, "Second nullptr-data call must return true");
    assert_true(jpeg.width == imgW && jpeg.height == imgH,
        "Dimensions changed on a non-start call");
}

/*--------------------------------------------------------------
 *  2. Simple 2×2 RGB conversion (BGR → RGB)
 *--------------------------------------------------------------*/
static void test_rgb_write_simple()
{
    const uint16_t W = 2, H = 2;
    rgb_jpg_decoder jpeg{};
    jpeg.data_offset = 0;
    jpeg.output = new uint8_t[W * H * RGB888_BYTES];
    std::memset(jpeg.output, 0, W * H * RGB888_BYTES);

    // initialise dimensions (required by the main loop)
    bool ok = _rgb_write(&jpeg, 0, 0, W, H, nullptr);
    assert_true(ok, "Init call failed");

    // source data – BGR order per pixel:
    //  (1,2,3) (4,5,6)
    //  (7,8,9) (10,11,12)
    std::vector<uint8_t> src = {
        1, 2, 3,   4, 5, 6,
        7, 8, 9,   10, 11, 12
    };
    // Expected output – RGB order:
    //  (3,2,1) (6,5,4)
    //  (9,8,7) (12,11,10)
    std::vector<uint8_t> expected = {
        3, 2, 1,   6, 5, 4,
        9, 8, 7,   12, 11, 10
    };

    ok = _rgb_write(&jpeg, 0, 0, W, H, src.data());
    assert_true(ok, "RGB conversion call returned false");
    assert_true(std::memcmp(jpeg.output, expected.data(),
        W * H * RGB888_BYTES) == 0,
        "RGB output does not match expected data");

    delete[] jpeg.output;
}

/*--------------------------------------------------------------
 *  3. Grayscale conversion (colorDepth != RGB888_BYTES)
 *--------------------------------------------------------------*/
static void test_grayscale_write()
{
    const uint16_t W = 2, H = 2;
    rgb_jpg_decoder jpeg{};
    jpeg.data_offset = 0;
    jpeg.output = new uint8_t[W * H * RGB888_BYTES]; // buffer is still sized for RGB
    std::memset(jpeg.output, 0, W * H * RGB888_BYTES);

    // initialise dimensions
    bool ok = _rgb_write(&jpeg, 0, 0, W, H, nullptr);
    assert_true(ok, "Init call failed");

    // Switch to grayscale mode
    colorDepth = 1; // any value different from RGB888_BYTES

    // Same source data as the RGB test (BGR order)
    std::vector<uint8_t> src = {
        1, 2, 3,   4, 5, 6,
        7, 8, 9,   10, 11, 12
    };
    // Expected grayscale values (average of R,G,B)
    // (1+2+3)/3 = 2, (4+5+6)/3 = 5, (7+8+9)/3 = 8, (10+11+12)/3 = 11
    uint8_t expectedGray[W * H] = { 2, 5, 8, 11 };

    ok = _rgb_write(&jpeg, 0, 0, W, H, src.data());
    assert_true(ok, "Grayscale conversion call returned false");

    // Verify each pixel using the same addressing logic that _rgb_write uses.
    // For stride == 1 the byte offset of pixel (row, col) is:
    //   row * (width * RGB888_BYTES) + col
    for (uint16_t row = 0; row < H; ++row) {
        for (uint16_t col = 0; col < W; ++col) {
            size_t offset = static_cast<size_t>(row) * jpeg.width * RGB888_BYTES + col;
            uint8_t got = jpeg.output[offset];
            size_t flatIdx = static_cast<size_t>(row) * W + col;
            assert_true(got == expectedGray[flatIdx],
                "Grayscale value mismatch at pixel index");
        }
    }

    delete[] jpeg.output;

    // Restore colour mode for the remaining tests
    colorDepth = RGB888_BYTES;
}

/*--------------------------------------------------------------
 *  4. Sub‑region write (write only a 2×2 block inside a 4×4 image)
 *--------------------------------------------------------------*/
static void test_subregion_write()
{
    const uint16_t IMG_W = 4, IMG_H = 4;
    const uint16_t REGION_W = 2, REGION_H = 2;
    const uint16_t X = 1, Y = 1; // top‑left corner of the region inside the 4×4 image

    rgb_jpg_decoder jpeg{};
    jpeg.data_offset = 0;
    jpeg.output = new uint8_t[IMG_W * IMG_H * RGB888_BYTES];
    std::memset(jpeg.output, 0xFF, IMG_W * IMG_H * RGB888_BYTES); // known pattern outside the region

    // initialise whole image dimensions
    bool ok = _rgb_write(&jpeg, 0, 0, IMG_W, IMG_H, nullptr);
    assert_true(ok, "Init call failed");

    // Build source data for the 2×2 block.
    // Scheme: B = row*10 + col, G = B+1, R = B+2.
    // (0,0) → B=0,G=1,R=2   (0,1) → B=1,G=2,R=3
    // (1,0) → B=10,G=11,R=12 (1,1) → B=11,G=12,R=13
    std::vector<uint8_t> src = {
        0, 1, 2,   1, 2, 3,
        10,11,12, 11,12,13
    };

    // Expected output after BGR→RGB conversion:
    // (2,1,0) (3,2,1)
    // (12,11,10) (13,12,11)
    std::vector<uint8_t> expectedBlock = {
        2,1,0,   3,2,1,
        12,11,10, 13,12,11
    };

    ok = _rgb_write(&jpeg, X, Y, REGION_W, REGION_H, src.data());
    assert_true(ok, "Sub-region write returned false");

    // Verify the whole image: the region must match expectedBlock,
    // everything else must stay at the initial 0xFF.
    for (uint16_t row = 0; row < IMG_H; ++row) {
        for (uint16_t col = 0; col < IMG_W; ++col) {
            size_t idx = (row * IMG_W + col) * RGB888_BYTES;
            if (row >= Y && row < Y + REGION_H && col >= X && col < X + REGION_W) {
                // Inside the region → compare with the reference block.
                size_t localRow = row - Y;
                size_t localCol = col - X;
                size_t expIdx = (localRow * REGION_W + localCol) * RGB888_BYTES;
                assert_true(std::memcmp(jpeg.output + idx,
                    expectedBlock.data() + expIdx,
                    RGB888_BYTES) == 0,
                    "Pixel inside region does not match expected value");
            }
            else {
                // Outside the region → original 0xFF must be untouched.
                for (int i = 0; i < RGB888_BYTES; ++i) {
                    assert_true(jpeg.output[idx + i] == 0xFF,
                        "Pixel outside region was unexpectedly modified");
                }
            }
        }
    }

    delete[] jpeg.output;
}

/*=====================================================================
 *  Benchmark – repeatedly convert a full‑HD frame and report the speed.
 *====================================================================*/

static void benchmark_rgb_write()
{
    constexpr uint16_t BENCH_WIDTH = 1920;
    constexpr uint16_t BENCH_HEIGHT = 1080;
    constexpr int      BENCH_ITERATIONS = 1000;   // how many frames to process

    std::cout << "\nBenchmark (RGB, " << BENCH_WIDTH << "x" << BENCH_HEIGHT
        << ", " << BENCH_ITERATIONS << " iterations):\n";

    rgb_jpg_decoder jpeg{};
    jpeg.width = BENCH_WIDTH;
    jpeg.height = BENCH_HEIGHT;
    jpeg.data_offset = 0;
    jpeg.output = new uint8_t[BENCH_WIDTH * BENCH_HEIGHT * RGB888_BYTES];
    std::memset(jpeg.output, 0, BENCH_WIDTH * BENCH_HEIGHT * RGB888_BYTES);

    // deterministic source data (simple ramp)
    std::vector<uint8_t> src(BENCH_WIDTH * BENCH_HEIGHT * RGB888_BYTES);
    for (size_t i = 0; i < src.size(); ++i) src[i] = static_cast<uint8_t>(i & 0xFF);

    // Warm‑up run (helps neutralise first‑call overhead)
    (void)_rgb_write(&jpeg, 0, 0, BENCH_WIDTH, BENCH_HEIGHT, src.data());

    auto t0 = std::chrono::high_resolution_clock::now();
    for (int i = 0; i < BENCH_ITERATIONS; ++i) {
        bool ok = _rgb_write(&jpeg, 0, 0, BENCH_WIDTH, BENCH_HEIGHT, src.data());
        assert_true(ok, "Benchmark iteration returned false");
    }
    auto t1 = std::chrono::high_resolution_clock::now();

    using ms = std::chrono::duration<double, std::milli>;
    double totalMs = std::chrono::duration_cast<ms>(t1 - t0).count();
    double avgMs = totalMs / BENCH_ITERATIONS;
    double bytesPerIter = static_cast<double>(BENCH_WIDTH) *
        BENCH_HEIGHT *
        RGB888_BYTES;
    double mbPerSec = (bytesPerIter * BENCH_ITERATIONS) / (totalMs / 1000.0) / (1024.0 * 1024.0);

    std::cout << "  Total time : " << static_cast<int>(totalMs) << " ms\n";
    std::cout << "  Avg. time  : " << avgMs << " ms per frame\n";
    std::cout << "  Throughput : " << static_cast<int>(mbPerSec) << " MB/s\n";

    delete[] jpeg.output;
}

/*=====================================================================
 *  Main – run the test suite and then the benchmark.
 *====================================================================*/

int main()
{
    std::cout << "=== _rgb_write unit tests ===\n";

    try {
        ++g_totalTests; test_init();                ++g_passedTests; std::cout << "Running test_init ... PASS\n";
        ++g_totalTests; test_rgb_write_simple();   ++g_passedTests; std::cout << "Running test_rgb_write_simple ... PASS\n";
        ++g_totalTests; test_grayscale_write();    ++g_passedTests; std::cout << "Running test_grayscale_write ... PASS\n";
        ++g_totalTests; test_subregion_write();    ++g_passedTests; std::cout << "Running test_subregion_write ... PASS\n";
    }
    catch (const std::exception& e) {
        std::cerr << "\nTest failed: " << e.what() << "\n";
        std::cerr << "Passed " << g_passedTests << " out of " << g_totalTests << " tests.\n";
        return 1;
    }

    std::cout << "\nAll " << g_passedTests << " tests passed.\n";

    // Run the small benchmark
    benchmark_rgb_write();

    return 0;
}
