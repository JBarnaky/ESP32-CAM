#define _CRT_SECURE_NO_WARNINGS
#include <iostream>
#include <string>
#include <vector>
#include <chrono>
#include <cstring>
#include <functional>
#include <iomanip>

// ============================================================================
// 1. MOCK ENVIRONMENT (Fixed for WiFi.STA syntax)
// ============================================================================

using esp_ping_handle_t = void*;
enum wl_status_t {
    WL_NO_SHIELD = 255,
    WL_NO_SSID_AVAIL = 254,
    WL_CONNECTED = 3,
    WL_CONNECT_FAILED = 4,
    WL_CONNECTION_LOST = 5
};

int netMode = 0;
bool usePing = true;
char ST_SSID[32] = "TestSSID";

struct MockSystem {
    std::vector<std::string> logs;
    int wifiRestartCount = 0;
    int ethRestartCount = 0;
    bool watchdogResetCalled = false;
    bool statusCheckCalled = false;
    wl_status_t currentWifiStatus = WL_CONNECTED;

    void reset() {
        logs.clear();
        wifiRestartCount = 0;
        ethRestartCount = 0;
        watchdogResetCalled = false;
        statusCheckCalled = false;
    }
} gMockSys;

#define LOG_WRN(msg) gMockSys.logs.push_back(std::string("WARN: ") + msg)

void resetWatchDog(int task, int timeoutMs) {
    gMockSys.watchdogResetCalled = true;
}

void startNetwork(bool force) {
    gMockSys.ethRestartCount++;
    LOG_WRN("Ethernet restart triggered");
}

void startWifi(bool force) {
    gMockSys.wifiRestartCount++;
    LOG_WRN("WiFi restart triggered");
}

bool netIsConnected() {
    return (gMockSys.currentWifiStatus == WL_CONNECTED);
}

void statusCheck() {
    gMockSys.statusCheckCalled = true;
    LOG_WRN("Status check performed");
}

// --- FIX STARTS HERE ---
struct MockWiFiStation {
    wl_status_t status() {
        return gMockSys.currentWifiStatus;
    }
};

struct MockWiFiClass {
    MockWiFiStation STA; // Enables WiFi.STA.status()
};

MockWiFiClass WiFi;
// --- FIX ENDS HERE ---

// ============================================================================
// 2. TARGET FUNCTION (Unchanged)
// ============================================================================

//static void pingTimeout(esp_ping_handle_t hdl, void* args) {
//    resetWatchDog(0, 10 * 1000 * 2);
//
//    if (netMode > 0) {
//        if (usePing) {
//            LOG_WRN("Failed to ping gateway, restart ethernet ...");
//            startNetwork(false);
//        }
//        else {
//            if (netIsConnected()) statusCheck();
//            else {
//                LOG_WRN("Disconnected, restart ethernet ...");
//                startNetwork(false);
//            }
//        }
//    }
//    else {
//        if (strlen(ST_SSID)) {
//            // This line now works because WiFi has the .STA member
//            wl_status_t wStat = WiFi.STA.status();
//
//            if (wStat != WL_NO_SSID_AVAIL && wStat != WL_NO_SHIELD) {
//                if (usePing) {
//                    LOG_WRN("Failed to ping gateway, restart wifi ...");
//                    startWifi(false);
//                }
//                else {
//                    if (wStat == WL_CONNECTED) statusCheck();
//                    else {
//                        LOG_WRN("Disconnected, restart wifi ...");
//                        startWifi(false);
//                    }
//                }
//            }
//        }
//    }
//}

// Handles ping timeout: resets watchdog, then handles network/wifi reconnection based on mode and status
static void pingTimeout(esp_ping_handle_t hdl, void* args) {
    (void)hdl; (void)args; // Suppress unused parameter warnings

    resetWatchDog(0, 20000); // Reset watchdog timer with 20-second timeout

    if (netMode > 0) { // Network mode active (ethernet-based)
        if (usePing) { LOG_WRN("Failed to ping gateway, restart ethernet ..."); startNetwork(false); return; } // Ping failed, restart ethernet
        if (netIsConnected()) { statusCheck(); return; } // Ethernet connected, perform status check
        LOG_WRN("Disconnected, restart ethernet ..."); // Ethernet disconnected, log warning
        startNetwork(false); // Restart ethernet connection
        return;
    }

    // WiFi mode (netMode == 0)
    if (ST_SSID[0] == '\0') { return; } // No SSID configured, exit early

    const wl_status_t wStat = WiFi.STA.status(); // Get current WiFi station status
    if (wStat == WL_NO_SSID_AVAIL || wStat == WL_NO_SHIELD) { return; } // WiFi unavailable, exit early

    if (usePing) { LOG_WRN("Failed to ping gateway, restart wifi ..."); startWifi(false); return; } // Ping failed, restart WiFi
    if (wStat == WL_CONNECTED) { statusCheck(); return; } // WiFi connected, perform status check
    LOG_WRN("Disconnected, restart wifi ..."); // WiFi disconnected, log warning
    startWifi(false); // Restart WiFi connection
}



// ============================================================================
// 3. UNIT TEST FRAMEWORK (No External Dependencies)
// ============================================================================

struct TestResult {
    std::string name;
    bool passed;
    std::string message;
};

std::vector<TestResult> testResults;

void runTest(const std::string& name, std::function<void()> setup, std::function<bool()> verify) {
    gMockSys.reset();
    setup();

    // Execute target function
    pingTimeout(nullptr, nullptr);

    bool success = verify();
    testResults.push_back({ name, success, success ? "PASS" : "FAIL" });

    std::cout << "[TEST] " << name << ": " << (success ? "PASS" : "FAIL") << std::endl;
    if (!success) {
        std::cout << "  Logs generated:" << std::endl;
        for (const auto& log : gMockSys.logs) std::cout << "    " << log << std::endl;
    }
}

void runAllTests() {
    std::cout << "\n--- Running Unit Tests ---\n" << std::endl;

    // Test 1: WiFi Mode, Ping Enabled, Ping Fails -> Should Restart WiFi
    runTest("WiFi_PingFail_Restart", []() {
        netMode = 0;
        usePing = true;
        strcpy(ST_SSID, "MyNetwork");
        gMockSys.currentWifiStatus = WL_CONNECTED; // Physically connected, but ping fails
        }, []() {
            return gMockSys.wifiRestartCount == 1 && gMockSys.logs.size() >= 2;
            });

        // Test 2: WiFi Mode, Ping Disabled, Connected -> Should Status Check Only
        runTest("WiFi_NoPing_Connected_StatusCheck", []() {
            netMode = 0;
            usePing = false;
            strcpy(ST_SSID, "MyNetwork");
            gMockSys.currentWifiStatus = WL_CONNECTED;
            }, []() {
                return gMockSys.wifiRestartCount == 0 &&
                    gMockSys.statusCheckCalled == true &&
                    gMockSys.logs.size() == 1 &&
                    gMockSys.logs[0] == "WARN: Status check performed";
                });

            // Test 3: WiFi Mode, Ping Disabled, Disconnected -> Should Restart
            runTest("WiFi_NoPing_Disconnected_Restart", []() {
                netMode = 0;
                usePing = false;
                strcpy(ST_SSID, "MyNetwork");
                gMockSys.currentWifiStatus = WL_CONNECT_FAILED;
                }, []() {
                    return gMockSys.wifiRestartCount == 1;
                    });

                // Test 4: Ethernet Mode, Ping Enabled -> Should Restart Ethernet
                runTest("Eth_PingFail_Restart", []() {
                    netMode = 1; // Ethernet
                    usePing = true;
                    }, []() {
                        return gMockSys.ethRestartCount == 1;
                        });

                    // Test 5: No SSID Configured -> Should do nothing
                    runTest("WiFi_NoSSID_Ignore", []() {
                        netMode = 0;
                        usePing = true;
                        strcpy(ST_SSID, ""); // Empty SSID
                        }, []() {
                            return gMockSys.wifiRestartCount == 0 && gMockSys.logs.empty();
                            });

                        // Test 6: Shield Missing (WL_NO_SHIELD) -> Should do nothing
                        runTest("WiFi_NoShield_Ignore", []() {
                            netMode = 0;
                            usePing = true;
                            strcpy(ST_SSID, "MyNetwork");
                            gMockSys.currentWifiStatus = WL_NO_SHIELD;
                            }, []() {
                                return gMockSys.wifiRestartCount == 0;
                                });
}

// ============================================================================
// 4. BENCHMARK SUITE
// ============================================================================

void runBenchmark() {
    std::cout << "\n--- Running Benchmark ---\n" << std::endl;

    // Setup stable environment
    netMode = 0;
    usePing = true;
    strcpy(ST_SSID, "BenchmarkSSID");
    gMockSys.currentWifiStatus = WL_CONNECTED;

    const int iterations = 100000;

    auto start = std::chrono::high_resolution_clock::now();

    for (int i = 0; i < iterations; ++i) {
        // Reset mocks inside loop to prevent vector growth affecting timing significantly
        // though in real HW, logs go to UART, not RAM vector. 
        // We simulate the 'happy path' logic cost.
        gMockSys.logs.clear();
        gMockSys.wifiRestartCount = 0;

        pingTimeout(nullptr, nullptr);
    }

    auto end = std::chrono::high_resolution_clock::now();
    std::chrono::duration<double, std::micro> elapsed = end - start;

    double avgTime = elapsed.count() / iterations;

    std::cout << "Iterations: " << iterations << std::endl;
    std::cout << "Total Time: " << elapsed.count() << " us" << std::endl;
    std::cout << "Average Time per Call: " << std::fixed << std::setprecision(2) << avgTime << " us" << std::endl;
    std::cout << "Note: Includes mock overhead (vector clear/push)." << std::endl;
}

// ============================================================================
// 5. MAIN ENTRY POINT
// ============================================================================

int main() {
    std::cout << "ESP32-CAM pingTimeout Emulation (Windows/VS2022)" << std::endl;
    std::cout << "Reference Issue: #221 (Ping failures despite good signal)" << std::endl;

    runAllTests();
    runBenchmark();

    std::cout << "\n--- Summary ---" << std::endl;
    int passed = 0;
    for (const auto& res : testResults) {
        if (res.passed) passed++;
    }
    std::cout << "Tests Passed: " << passed << "/" << testResults.size() << std::endl;

    return 0;
}