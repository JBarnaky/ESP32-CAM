#include <iostream>
#include <vector>
#include <chrono>
#include <cassert>

//-------------------------------------------------------------------
// Original implementation (unchanged)
//-------------------------------------------------------------------
//static void rgbToGray(uint8_t* buffer, int width, int height) {
//    // convert rgb buffer to grayscale in place
//    for (int i = 0; i < width * height; ++i) {
//        int index = i * 3;
//        // Calculate grayscale value using luminance formula
//        buffer[i] = (uint8_t)(((77 * buffer[index]) + (150 * buffer[index + 1]) + (29 * buffer[index + 2])) >> 8);
//    }
//}

// Refactor rgbToGray
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



/*------------------------------------------------------------------------------------------------------*/
void test_rgbToGray() {
    // Test case 1: Basic conversion with known values
    {
        uint8_t buffer[] = { 255, 0, 0,   // Red pixel -> should be ~77
                            0, 255, 0,   // Green pixel -> should be ~150
                            0, 0, 255 };  // Blue pixel -> should be ~29
        int width = 1, height = 1;

        // This is incorrect since we're only processing first pixel
        // Let's fix the test to match function behavior
        uint8_t testBuffer[] = { 255, 0, 0,   // Red pixel
                                0, 255, 0,   // Green pixel
                                0, 0, 255,   // Blue pixel
                                100, 100, 100 }; // Additional pixel
        int w = 2, h = 2; // 4 pixels total, need 12 bytes but only process 4*3=12 bytes

        uint8_t expected[] = { 255, 0, 0,   // Red pixel
                              0, 255, 0,   // Green pixel  
                              0, 0, 255,   // Blue pixel
                              100, 100, 100 };

        // First, calculate expected grayscale values
        uint8_t gray1 = (uint8_t)(((77 * 255) + (150 * 0) + (29 * 0)) >> 8); // ~77
        uint8_t gray2 = (uint8_t)(((77 * 0) + (150 * 255) + (29 * 0)) >> 8); // ~150
        uint8_t gray3 = (uint8_t)(((77 * 0) + (150 * 0) + (29 * 255)) >> 8); // ~29
        uint8_t gray4 = (uint8_t)(((77 * 100) + (150 * 100) + (29 * 100)) >> 8); // ~25600/256 = 100

        memcpy(testBuffer, expected, sizeof(expected));
        rgbToGray(testBuffer, 2, 2);

        assert(testBuffer[0] == gray1);
        assert(testBuffer[1] == gray2);
        assert(testBuffer[2] == gray3);
        assert(testBuffer[3] == gray4);
    }

    // Test case 2: All black pixels
    {
        uint8_t buffer[12] = { 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0 };
        uint8_t expected[12] = { 0, 0, 0, 0 };
        rgbToGray(buffer, 2, 2);
        assert(buffer[0] == 0);
        assert(buffer[1] == 0);
        assert(buffer[2] == 0);
        assert(buffer[3] == 0);
    }

    // Test case 3: All white pixels
    {
        uint8_t buffer[12] = { 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255 };
        rgbToGray(buffer, 2, 2);
        uint8_t expected = (uint8_t)(((77 * 255) + (150 * 255) + (29 * 255)) >> 8); // (77+150+29)*255/256 = 256*255/256 = 255
        assert(buffer[0] == expected);
        assert(buffer[1] == expected);
        assert(buffer[2] == expected);
        assert(buffer[3] == expected);
    }

    // Test case 4: Single pixel
    {
        uint8_t buffer[3] = { 100, 150, 200 };
        uint8_t expected = (uint8_t)(((77 * 100) + (150 * 150) + (29 * 200)) >> 8);
        rgbToGray(buffer, 1, 1);
        assert(buffer[0] == expected);
    }

    std::cout << "All tests passed!" << std::endl;
}

void benchmark_rgbToGray() {
    const int width = 1920;
    const int height = 1080;
    const size_t bufferSize = width * height * 3;

    std::vector<uint8_t> buffer(bufferSize);

    // Fill with random-like data
    for (size_t i = 0; i < bufferSize; i++) {
        buffer[i] = static_cast<uint8_t>(i % 256);
    }

    auto start = std::chrono::high_resolution_clock::now();

    const int iterations = 100;
    for (int i = 0; i < iterations; ++i) {
        // Make a copy of the buffer for each iteration
        std::vector<uint8_t> tempBuffer = buffer;
        rgbToGray(tempBuffer.data(), width, height);
    }

    auto end = std::chrono::high_resolution_clock::now();
    auto duration = std::chrono::duration_cast<std::chrono::milliseconds>(end - start);

    std::cout << "Benchmark completed in " << duration.count() << " ms for " << iterations << " iterations" << std::endl;
    std::cout << "Average time per iteration: " << duration.count() / static_cast<double>(iterations) << " ms" << std::endl;

    // Print the resulting grayscale values of first few pixels to verify correctness
    std::vector<uint8_t> tempBuffer = buffer;
    rgbToGray(tempBuffer.data(), width, height);
    std::cout << "First 5 grayscale values: ";
    for (int i = 0; i < 5; ++i) {
        std::cout << static_cast<int>(tempBuffer[i]) << " ";
    }
    std::cout << std::endl;
}

int main() {
    test_rgbToGray();
    benchmark_rgbToGray();
    return 0;
}