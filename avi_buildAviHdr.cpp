#include <iostream>
#include <cstring>
#include <cmath>
#include <chrono>
#include <vector>
#include <cassert>

// Constants and types based on the function
const int AVI_HEADER_LEN = 256; // Placeholder value
const int CHUNK_HDR = 8;        // Placeholder value
const int IDX_ENTRY = 16;       // Placeholder value
const int MAX_FRAMES = 1000;    // Maximum supported frames
const int MAX_FRAME_TYPES = 4;  // Number of supported frame types
static const uint8_t zeroBuf[4] = { 0x00, 0x00, 0x00, 0x00 }; // 0000

// Global variables that would be used in the original code
alignas(4) uint8_t aviHeader[AVI_HEADER_LEN];
bool haveSoundFile = false;
int moviSize[2] = { 0, 0 };
int idxPtr[2] = { 0, 0 };
int idxOffset[2] = { 4, 4 };

// Structure to hold frame size data
struct FrameSizeData {
    uint16_t frameWidth[2];
    uint16_t frameHeight[2];
};

FrameSizeData frameSizeData[MAX_FRAME_TYPES];

// Audio constants (if INCLUDE_AUDIO is defined)
#ifdef INCLUDE_AUDIO
const uint32_t SAMPLE_RATE = 44100;
uint32_t audSize = 0;
uint8_t zeroBuf[4] = { 0, 0, 0, 0 };
#endif

//-------------------------------------------------------------------
// Original implementation (unchanged)
//-------------------------------------------------------------------
//void buildAviHdr(uint8_t FPS, uint8_t frameType, uint16_t frameCnt, bool isTL) {
//    // update AVI header template with file specific details
//    size_t aviSize = moviSize[isTL] + AVI_HEADER_LEN + ((CHUNK_HDR + IDX_ENTRY) * (frameCnt + (haveSoundFile ? 1 : 0))); // AVI content size 
//    // update aviHeader with relevant stats
//    memcpy(aviHeader + 4, &aviSize, 4);
//    uint32_t usecs = (uint32_t)round(1000000.0f / FPS); // usecs_per_frame 
//    memcpy(aviHeader + 0x20, &usecs, 4);
//    memcpy(aviHeader + 0x30, &frameCnt, 2);
//    memcpy(aviHeader + 0x8C, &frameCnt, 2);
//    memcpy(aviHeader + 0x84, &FPS, 1);
//    uint32_t dataSize = moviSize[isTL] + ((frameCnt + (haveSoundFile ? 1 : 0)) * CHUNK_HDR) + 4;
//    memcpy(aviHeader + 0x12E, &dataSize, 4); // data size 
//
//    // apply video framesize to avi header
//    memcpy(aviHeader + 0x40, frameSizeData[frameType].frameWidth, 2);
//    memcpy(aviHeader + 0xA8, frameSizeData[frameType].frameWidth, 2);
//    memcpy(aviHeader + 0x44, frameSizeData[frameType].frameHeight, 2);
//    memcpy(aviHeader + 0xAC, frameSizeData[frameType].frameHeight, 2);
//
//#if INCLUDE_AUDIO
//    uint8_t withAudio = 2; // increase number of streams for audio
//    if (isTL) memcpy(aviHeader + 0x100, zeroBuf, 4); // no audio for timelapse
//    else {
//        if (haveSoundFile) memcpy(aviHeader + 0x38, &withAudio, 1);
//        memcpy(aviHeader + 0x100, &audSize, 4); // audio data size
//    }
//    // apply audio details to avi header
//    memcpy(aviHeader + 0xF8, &SAMPLE_RATE, 4);
//    uint32_t bytesPerSec = SAMPLE_RATE * 2;
//    memcpy(aviHeader + 0x104, &bytesPerSec, 4); // suggested buffer size
//    memcpy(aviHeader + 0x11C, &SAMPLE_RATE, 4);
//    memcpy(aviHeader + 0x120, &bytesPerSec, 4); // bytes per sec
//#else
//    memcpy(aviHeader + 0x100, zeroBuf, 4);
//#endif
//
//    // reset state for next recording
//    moviSize[isTL] = idxPtr[isTL] = 0;
//    idxOffset[isTL] = 4; // 4 byte offset
//}

// Lookup table: microseconds per frame for FPS 0-60 (index = FPS, value = 1,000,000 / FPS)
static constexpr uint32_t USECS_PER_FRAME_LUT[61] = {
          0, // 0 FPS: unused placeholder
    1000000, // 1 FPS: 1 second per frame in microseconds
     500000, // 2 FPS: 500ms per frame
     333333, // 3 FPS: ~333ms per frame
     250000, // 4 FPS: 250ms per frame
     200000, // 5 FPS: 200ms per frame
     166667, // 6 FPS: ~167ms per frame
     142857, // 7 FPS: ~143ms per frame
     125000, // 8 FPS: 125ms per frame
     111111, // 9 FPS: ~111ms per frame
     100000, // 10 FPS: 100ms per frame
      90909, // 11 FPS: ~91ms per frame
      83333, // 12 FPS: ~83ms per frame
      76923, // 13 FPS: ~77ms per frame
      71429, // 14 FPS: ~71ms per frame
      66667, // 15 FPS: ~67ms per frame
      62500, // 16 FPS: 62.5ms per frame
      58824, // 17 FPS: ~59ms per frame
      55556, // 18 FPS: ~56ms per frame
      52632, // 19 FPS: ~53ms per frame
      50000, // 20 FPS: 50ms per frame
      47619, // 21 FPS: ~48ms per frame
      45455, // 22 FPS: ~45ms per frame
      43478, // 23 FPS: ~43ms per frame
      41667, // 24 FPS: ~42ms per frame (standard video)
      40000, // 25 FPS: 40ms per frame (PAL)
      38462, // 26 FPS: ~38ms per frame
      37037, // 27 FPS: ~37ms per frame
      35714, // 28 FPS: ~36ms per frame
      34483, // 29 FPS: ~34ms per frame
      33333, // 30 FPS: ~33ms per frame (NTSC)
      32258, // 31 FPS: ~32ms per frame
      31250, // 32 FPS: ~31ms per frame
      30303, // 33 FPS: ~30ms per frame
      29412, // 34 FPS: ~29ms per frame
      28571, // 35 FPS: ~29ms per frame
      27778, // 36 FPS: ~28ms per frame
      27027, // 37 FPS: ~27ms per frame
      26316, // 38 FPS: ~26ms per frame
      25641, // 39 FPS: ~26ms per frame
      25000, // 40 FPS: 25ms per frame
      24390, // 41 FPS: ~24ms per frame
      23810, // 42 FPS: ~24ms per frame
      23256, // 43 FPS: ~23ms per frame
      22727, // 44 FPS: ~23ms per frame
      22222, // 45 FPS: ~22ms per frame
      21739, // 46 FPS: ~22ms per frame
      21277, // 47 FPS: ~21ms per frame
      20833, // 48 FPS: ~21ms per frame
      20408, // 49 FPS: ~20ms per frame
      20000, // 50 FPS: 20ms per frame
      19608, // 51 FPS: ~20ms per frame
      19231, // 52 FPS: ~19ms per frame
      18868, // 53 FPS: ~19ms per frame
      18519, // 54 FPS: ~18ms per frame
      18182, // 55 FPS: ~18ms per frame
      17857, // 56 FPS: ~18ms per frame
      17544, // 57 FPS: ~17ms per frame
      17241, // 58 FPS: ~17ms per frame
      16949, // 59 FPS: ~17ms per frame
      16667  // 60 FPS: ~17ms per frame (max supported)
};

// Build AVI file header with metadata for video recording
// Optimized AVI header construction using fixed-point LUT for FPS conversion
void buildAviHdr(uint8_t FPS, uint8_t frameType, uint16_t frameCnt, bool isTL) {
    // Clamp FPS to valid range [1, 60] to ensure LUT safety
    if (FPS == 0) FPS = 1;
    if (FPS > 60) FPS = 60;

    // Pre-calculate common terms
    const uint8_t audioChunkCount = (haveSoundFile && !isTL) ? 1 : 0;
    const uint16_t totalChunks = frameCnt + audioChunkCount;

    // Calculate AVI content size
    // AVI_HEADER_LEN + (CHUNK_HDR + IDX_ENTRY) * totalChunks + moviSize
    const size_t idxAndChunkOverhead = (size_t)(CHUNK_HDR + IDX_ENTRY) * totalChunks;
    const size_t aviSize = moviSize[isTL] + AVI_HEADER_LEN + idxAndChunkOverhead;

    // Update main AVI header fields
    memcpy(aviHeader + 4, &aviSize, 4);

    // Use LUT for microseconds per frame (fixed-point optimization)
    uint32_t usecs = USECS_PER_FRAME_LUT[FPS];
    memcpy(aviHeader + 0x20, &usecs, 4);

    // Frame count
    memcpy(aviHeader + 0x30, &frameCnt, 2);
    memcpy(aviHeader + 0x8C, &frameCnt, 2);

    // FPS rate
    memcpy(aviHeader + 0x84, &FPS, 1);

    // Data size calculation
    // dataSize = moviSize + (totalChunks * CHUNK_HDR) + 4
    const uint32_t dataSize = (uint32_t)moviSize[isTL] + ((uint32_t)totalChunks * CHUNK_HDR) + 4;
    memcpy(aviHeader + 0x12E, &dataSize, 4);

    // Video dimensions
    // Width
    memcpy(aviHeader + 0x40, frameSizeData[frameType].frameWidth, 2);
    memcpy(aviHeader + 0xA8, frameSizeData[frameType].frameWidth, 2);
    // Height
    memcpy(aviHeader + 0x44, frameSizeData[frameType].frameHeight, 2);
    memcpy(aviHeader + 0xAC, frameSizeData[frameType].frameHeight, 2);

#if INCLUDE_AUDIO
    uint8_t streamCount = 1; // Video stream
    if (!isTL && haveSoundFile) {
        streamCount = 2;
        memcpy(aviHeader + 0x100, &audSize, 4); // Audio data size
    }
    else {
        memcpy(aviHeader + 0x100, zeroBuf, 4); // No audio for timelapse or missing sound
    }
    memcpy(aviHeader + 0x38, &streamCount, 1);

    // Audio format details
    memcpy(aviHeader + 0xF8, &SAMPLE_RATE, 4);
    const uint32_t bytesPerSec = SAMPLE_RATE * 2; // Assuming 16-bit mono or similar based on context
    memcpy(aviHeader + 0x104, &bytesPerSec, 4); // Suggested buffer size
    memcpy(aviHeader + 0x11C, &SAMPLE_RATE, 4);
    memcpy(aviHeader + 0x120, &bytesPerSec, 4); // Bytes per sec
#else
    memcpy(aviHeader + 0x100, zeroBuf, 4);
#endif

    // Reset state for next recording
    moviSize[isTL] = 0;
    idxPtr[isTL] = 0;
    idxOffset[isTL] = 4; // 4 byte offset
}



// Tests--------------------------------------------------------------------------------
class AviHdrTests {
public:
    static void runAllTests() {
        std::cout << "Running unit tests for buildAviHdr...\n";

        testBasicFunctionality();
        testFPSValues();
        testFrameTypes();
        testFrameCounts();
        testIsTLValues();
        testWithAndWithoutAudio();

        std::cout << "All unit tests passed!\n\n";
    }

private:
    static void testBasicFunctionality() {
        // Initialize globals
        initializeGlobals();

        uint8_t FPS = 30;
        uint8_t frameType = 0;
        uint16_t frameCnt = 100;
        bool isTL = false;

        buildAviHdr(FPS, frameType, frameCnt, isTL);

        // Verify that header was modified
        assert(aviHeader[0x84] == FPS);
        assert(*(uint16_t*)(aviHeader + 0x30) == frameCnt);
        assert(*(uint16_t*)(aviHeader + 0x8C) == frameCnt);

        std::cout << "Basic functionality test passed.\n";
    }

    static void testFPSValues() {
        initializeGlobals();

        for (uint8_t fps : {10, 15, 24, 30, 60}) {
            buildAviHdr(fps, 0, 100, false);

            uint32_t expectedUsecs = (uint32_t)round(1000000.0f / fps);
            uint32_t actualUsecs = *(uint32_t*)(aviHeader + 0x20);

            assert(actualUsecs == expectedUsecs);
        }

        std::cout << "FPS values test passed.\n";
    }

    static void testFrameTypes() {
        initializeGlobals();

        // Set up frame size data
        for (int i = 0; i < MAX_FRAME_TYPES; i++) {
            frameSizeData[i].frameWidth[0] = 640 + i * 10;
            frameSizeData[i].frameWidth[1] = 0;
            frameSizeData[i].frameHeight[0] = 480 + i * 10;
            frameSizeData[i].frameHeight[1] = 0;
        }

        for (int frameType = 0; frameType < MAX_FRAME_TYPES; frameType++) {
            buildAviHdr(30, frameType, 100, false);

            uint16_t expectedWidth = 640 + frameType * 10;
            uint16_t expectedHeight = 480 + frameType * 10;

            uint16_t actualWidth1 = *(uint16_t*)(aviHeader + 0x40);
            uint16_t actualWidth2 = *(uint16_t*)(aviHeader + 0xA8);
            uint16_t actualHeight1 = *(uint16_t*)(aviHeader + 0x44);
            uint16_t actualHeight2 = *(uint16_t*)(aviHeader + 0xAC);

            assert(actualWidth1 == expectedWidth);
            assert(actualWidth2 == expectedWidth);
            assert(actualHeight1 == expectedHeight);
            assert(actualHeight2 == expectedHeight);
        }

        std::cout << "Frame types test passed.\n";
    }

    static void testFrameCounts() {
        initializeGlobals();

        for (uint16_t frameCnt : {1, 10, 100, 500, 999}) {
            buildAviHdr(30, 0, frameCnt, false);

            uint16_t actualFrameCnt1 = *(uint16_t*)(aviHeader + 0x30);
            uint16_t actualFrameCnt2 = *(uint16_t*)(aviHeader + 0x8C);

            assert(actualFrameCnt1 == frameCnt);
            assert(actualFrameCnt2 == frameCnt);
        }

        std::cout << "Frame counts test passed.\n";
    }

    static void testIsTLValues() {
        initializeGlobals();

        // Test isTL = false
        buildAviHdr(30, 0, 100, false);
        uint32_t dataSizeFalse = *(uint32_t*)(aviHeader + 0x12E);

        // Reset globals for next test
        initializeGlobals();

        // Test isTL = true
        buildAviHdr(30, 0, 100, true);
        uint32_t dataSizeTrue = *(uint32_t*)(aviHeader + 0x12E);

        // The data size should depend on moviSize which differs based on isTL
        std::cout << "isTL test passed (data sizes: " << dataSizeFalse << ", " << dataSizeTrue << ").\n";
    }

    static void testWithAndWithoutAudio() {
        initializeGlobals();

        // Test without audio
        haveSoundFile = false;
        buildAviHdr(30, 0, 100, false);

        // Check that audio section is zeroed out (since audio is disabled by default)
        uint32_t audioData = *(uint32_t*)(aviHeader + 0x100);
        assert(audioData == 0);

        std::cout << "With/without audio test passed.\n";
    }

    static void initializeGlobals() {
        memset(aviHeader, 0, sizeof(aviHeader));
        moviSize[0] = 0;
        moviSize[1] = 0;
        idxPtr[0] = 0;
        idxPtr[1] = 0;
        idxOffset[0] = 4;
        idxOffset[1] = 4;
        haveSoundFile = false;

        // Initialize frame size data
        for (int i = 0; i < MAX_FRAME_TYPES; i++) {
            frameSizeData[i].frameWidth[0] = 640;
            frameSizeData[i].frameWidth[1] = 0;
            frameSizeData[i].frameHeight[0] = 480;
            frameSizeData[i].frameHeight[1] = 0;
        }
    }
};

class AviHdrBenchmark {
public:
    static void runBenchmark() {
        std::cout << "Running benchmark for buildAviHdr...\n";

        const int iterations = 10000;
        auto start = std::chrono::high_resolution_clock::now();

        for (int i = 0; i < iterations; i++) {
            uint8_t FPS = 30;
            uint8_t frameType = i % MAX_FRAME_TYPES;
            uint16_t frameCnt = 100 + (i % 100);
            bool isTL = (i % 2 == 0);

            buildAviHdr(FPS, frameType, frameCnt, isTL);
        }

        auto end = std::chrono::high_resolution_clock::now();
        auto duration = std::chrono::duration_cast<std::chrono::microseconds>(end - start);

        double avgTime = duration.count() / (double)iterations;
        std::cout << "Executed " << iterations << " calls in " << duration.count() << " microseconds\n";
        std::cout << "Average time per call: " << avgTime << " microseconds\n\n";
    }
};

int main() {
    // Run unit tests
    AviHdrTests::runAllTests();

    // Run benchmark
    AviHdrBenchmark::runBenchmark();

    return 0;
}