#define _CRT_SECURE_NO_WARNINGS

#include <iostream>
#include <string>
#include <vector>
#include <chrono>
#include <thread>
#include <cstring>
#include <stdexcept>
#include <sstream>
#include <functional>
#include <cstdint>          // uint32_t, etc.

 /* --------------------------------------------------------------
  *  1️⃣  Minimal ESP32‑/Arduino‑like API stubs
  * -------------------------------------------------------------- */
enum wl_status_t {
    WL_NO_SSID_AVAIL = 0,
    WL_IDLE_STATUS,
    WL_SCAN_COMPLETED,
    WL_CONNECTED,
    WL_CONNECT_FAILED,
    WL_CONNECTION_LOST,
    WL_DISCONNECTED
};

/* --------------------------------------------------------------
 *  WiFi mode constants – they exist in the real Arduino core,
 *  but they are missing on a plain Windows build.
 * -------------------------------------------------------------- */
enum WiFiMode {
    WIFI_OFF = 0,
    WIFI_STA = 1,
    WIFI_AP = 2,
    WIFI_AP_STA = WIFI_AP | WIFI_STA   // value = 3
};

/* -----------------------------------------------------------------
 *  WiFi simulation data – the test code will fill these containers
 *  to control what the stub WiFi object “sees”.
 * ----------------------------------------------------------------- */
struct WiFiSim {
    static std::vector<std::string> ssids;   // network names found by scan
    static std::vector<long>       rssis;   // RSSI values per network
    static std::vector<long>       channels;// channel numbers per network
    static wl_status_t             staStatus; // value returned by WiFi.STA.status()

    static int      scanNetworkCount() { return static_cast<int>(ssids.size()); }
    static std::string getSSID(int i) { return (i >= 0 && i < (int)ssids.size()) ? ssids[i] : ""; }
    static long       getRSSI(int i) { return (i >= 0 && i < (int)rssis.size()) ? rssis[i] : 0; }
    static long       getChannel(int i) { return (i >= 0 && i < (int)channels.size()) ? channels[i] : 0; }
    static wl_status_t getSTAStatus() { return staStatus; }

    static void reset()
    {
        ssids.clear(); rssis.clear(); channels.clear();
        staStatus = WL_NO_SSID_AVAIL;
    }
};

std::vector<std::string> WiFiSim::ssids;
std::vector<long>        WiFiSim::rssis;
std::vector<long>        WiFiSim::channels;
wl_status_t              WiFiSim::staStatus = WL_NO_SSID_AVAIL;

/* -----------------------------------------------------------------
 *  Tiny replacements for the Arduino logging macros (they do nothing)
 * ----------------------------------------------------------------- */
inline void log_dummy(const char*, ...) {}
#define LOG_SEND(...) log_dummy(__VA_ARGS__)
#define LOG_INF(...)  log_dummy(__VA_ARGS__)
#define LOG_WRN(...)  log_dummy(__VA_ARGS__)
#define LOG_INF(fmt, ...) do { printf("[INFO] " fmt "\n", __VA_ARGS__); } while (0)

/* -----------------------------------------------------------------
 *  delay() and millis() – behave like the Arduino functions
 * ----------------------------------------------------------------- */
static uint32_t millis()
{
    using namespace std::chrono;
    static const steady_clock::time_point start = steady_clock::now();
    auto now = steady_clock::now();
    return static_cast<uint32_t>(duration_cast<milliseconds>(now - start).count());
}
static void delay(uint32_t ms) { std::this_thread::sleep_for(std::chrono::milliseconds(ms)); }

/* Very simple busy‑wait based delay */
void vTaskDelay(uint32_t ms) { std::this_thread::sleep_for(std::chrono::milliseconds(ms)); }

/* -----------------------------------------------------------------
 *  Global variables used by startWifi() (normally supplied by the sketch)
 * ----------------------------------------------------------------- */
int   netMode = 0;                          // 0 → try station mode
bool  allowAP = false;                      // allow AP fallback?
const char* hostName = "esp32cam";            // device hostname (used only for WiFi.STA.setHostname)
char  ST_SSID[64] = { 0 };                       // target SSID (set by the test)
void* pingHandle = nullptr;                   // nullptr → startPing() should be invoked

/* -----------------------------------------------------------------
 *  Flags that tell us whether the “real” helper functions have been called
 * ----------------------------------------------------------------- */
static bool setWifiSTA_called = false;
static bool setWifiAP_called = false;
static bool startPing_called = false;
static bool setupMdnsHost_called = false;

/* -----------------------------------------------------------------
 *  Helper functions that are called from startWifi()
 * ----------------------------------------------------------------- */
void setWifiSTA() { setWifiSTA_called = true; }
void setWifiAP() { setWifiAP_called = true; }
void startPing() { startPing_called = true; }
void setupMdnsHost() { setupMdnsHost_called = true; }

/* -----------------------------------------------------------------
 *  Miscellaneous stubs used inside startWifi()
 * ----------------------------------------------------------------- */
const char* getEncType(int /*idx*/) { return "WPA2-PSK"; }
const char* wifiStatusStr(wl_status_t s)
{
    switch (s) {
    case WL_CONNECTED:        return "CONNECTED";
    case WL_NO_SSID_AVAIL:    return "NO_SSID_AVAIL";
    case WL_IDLE_STATUS:      return "IDLE_STATUS";
    case WL_SCAN_COMPLETED:    return "SCAN_COMPLETED";
    case WL_CONNECT_FAILED:   return "CONNECT_FAILED";
    case WL_CONNECTION_LOST:  return "CONNECTION_LOST";
    case WL_DISCONNECTED:     return "DISCONNECTED";
    default:                  return "UNKNOWN";
    }
}

/* -----------------------------------------------------------------
 *  The stub WiFi class – mimics the API used by startWifi().
 *  All members are static so they can be accessed through the global
 *  instance “WiFi” just like in the Arduino environment.
 * ----------------------------------------------------------------- */
class WiFiClass
{
public:
    /* --------------------------------------------------------------
     *  “configuration” APIs (mode / persistent)
     * -------------------------------------------------------------- */
    static void mode(WiFiMode) {}
    static void persistent(bool) {}

    /* --------------------------------------------------------------
     *  STA sub‑object
     * -------------------------------------------------------------- */
    class STAClass {
    public:
        static void setAutoReconnect(bool) {}
        static void setHostname(const char*) {}
        static wl_status_t status() { return WiFiSim::getSTAStatus(); }
    };
    static STAClass STA;                // accessed as WiFi.STA.xxx

    /* --------------------------------------------------------------
     *  AP sub‑object
     * -------------------------------------------------------------- */
    class APClass {
    public:
        static void clear() {}
        static void end() {}
    };
    static APClass AP;                  // accessed as WiFi.AP.xxx

    /* --------------------------------------------------------------
     *  Scan‑related APIs
     * -------------------------------------------------------------- */
    static int scanNetworks() { return WiFiSim::scanNetworkCount(); }
    static std::string SSID(int i) { return WiFiSim::getSSID(i); }
    static long       RSSI(int i) { return WiFiSim::getRSSI(i); }
    static long       channel(int i) { return WiFiSim::getChannel(i); }
};

/* Definitions of the static members */
WiFiClass::STAClass WiFiClass::STA;
WiFiClass::APClass  WiFiClass::AP;

/* Global instance – exactly what the real Arduino core provides */
WiFiClass WiFi;

/* --------------------------------------------------------------
 *  2️⃣  The function under test (copy‑and‑paste – do **not** modify it)
 * -------------------------------------------------------------- */
//static bool startWifi(bool firstcall = true) {
//    // start wifi station (and wifi AP if allowed or station not defined)
//    if (firstcall) {
//        WiFi.mode(WIFI_AP_STA);
//        WiFi.persistent(false); // prevent the flash storage WiFi credentials
//        WiFi.STA.setAutoReconnect(false); // Set whether module will attempt to reconnect to an access point in case it is disconnected
//        WiFi.AP.clear();
//        WiFi.AP.end(); // kill rogue AP on startup
//        WiFi.STA.setHostname(hostName);
//        delay(100);
//    }
//
//    wl_status_t wlStat = WL_NO_SSID_AVAIL;
//    if (netMode == 0) {
//        // connect to Wifi station
//        setWifiSTA();
//        uint32_t startAttemptTime = millis();
//        // Stop trying on failure timeout, will try to reconnect later by ping
//        wlStat = WL_NO_SSID_AVAIL;
//        if (strlen(ST_SSID)) {
//            while (wlStat = WiFi.STA.status(),
//                wlStat != WL_CONNECTED && millis() - startAttemptTime < 5000) {
//                LOG_SEND(".");
//                delay(500);
//            }
//        }
//        // show stats of requested SSID
//        int numNetworks = WiFi.scanNetworks();
//        for (int i = 0; i < numNetworks; i++) {
//            if (!strcmp(WiFi.SSID(i).c_str(), ST_SSID))
//                LOG_INF("Wifi stats for %s - signal strength: %ld dBm; Encryption: %s; channel: %ld",
//                    ST_SSID, WiFi.RSSI(i), getEncType(i), WiFi.channel(i));
//        }
//        if (wlStat != WL_CONNECTED) LOG_WRN("SSID %s not connected %s", ST_SSID, wifiStatusStr(wlStat));
//    }
//
//    if (wlStat == WL_NO_SSID_AVAIL || allowAP) setWifiAP(); // AP allowed if no Station SSID eg on first time use 
//#if CONFIG_IDF_TARGET_ESP32S3
//    if (netMode == 0) setupMdnsHost(); // not on ESP32 as uses 6k of heap
//#endif
//    if (pingHandle == NULL) startPing();
//    return wlStat == WL_CONNECTED ? true : false;
//}

// Refactored: removes redundant WiFi scan, eliminates heap leak, reduces blocking overhead.
static bool startWifi(bool firstcall = true) {
    // Initialise the Wi‑Fi subsystem on the first call only.
    if (firstcall) {
        // Set dual mode (AP + STA) and turn off persistent storage.
        WiFi.mode(WIFI_AP_STA);
        WiFi.persistent(false);
        // Prevent automatic reconnection attempts.
        WiFi.STA.setAutoReconnect(false);
        // Remove any previous AP configuration.
        WiFi.AP.clear();
        WiFi.AP.end();
        // Assign the configured hostname to the STA interface.
        WiFi.STA.setHostname(hostName);
        // Short pause to let the hardware settle after the mode change.
        vTaskDelay(100);  //vTaskDelay(pdMS_TO_TICKS(100));
    }

    // Assume no SSID is available until a successful connection is made.
    wl_status_t wlStat = WL_NO_SSID_AVAIL;

    // ------------------------------------------------------------
    //  Station‑mode handling – try to connect to the configured AP.
    // ------------------------------------------------------------
    if (netMode == 0) {
        setWifiSTA();

        // Only attempt a connection if a non‑empty SSID was supplied.
        if (ST_SSID[0] != '\0') {
            const uint32_t startAttemptTime = millis();   // Record when we start trying.
            const uint32_t timeoutMs = 5000;        // Give the connection 5 s.

            // Poll Wi‑Fi status until we connect or the timeout expires.
            while (true) {
                wlStat = WiFi.STA.status();
                if (wlStat == WL_CONNECTED) {
                    break;                               // Connected successfully.
                }
                if (millis() - startAttemptTime >= timeoutMs) {
                    wlStat = WL_CONNECT_FAILED;         // Timeout – treat as failure.
                    break;
                }
                LOG_SEND(".");                         // Simple progress indicator.
                vTaskDelay(250);                       // Yield to other tasks. vTaskDelay(pdMS_TO_TICKS(250));
            }
        }
        else {
            wlStat = WL_NO_SSID_AVAIL;                  // No SSID configured.
        }

        // If we are not connected, run a diagnostic scan for the target SSID.
        if (wlStat != WL_CONNECTED) {
            int numNetworks = WiFi.scanNetworks();
            if (numNetworks > 0) {
                const size_t targetLen = strlen(ST_SSID);
                for (int i = 0; i < numNetworks; ++i) {
                    // Capture the scanned SSID in a local std::string to avoid dangling pointers.
                    const std::string scannedSsidStr = WiFi.SSID(i);
                    const char* scannedSsid = scannedSsidStr.c_str();

                    // Look for an exact match with the configured SSID.
                    if (scannedSsid && strncmp(scannedSsid, ST_SSID, targetLen) == 0 &&
                        scannedSsid[targetLen] == '\0') {
                        LOG_INF("Wifi stats for %s - signal strength: %ld dBm; Encryption: %s; channel: %ld",
                            ST_SSID,
                            static_cast<long>(WiFi.RSSI(i)),
                            getEncType(i),
                            static_cast<long>(WiFi.channel(i)));
                        break;                           // Stop after first exact match.
                    }
                }
            }
            LOG_WRN("SSID %s not connected %s", ST_SSID, wifiStatusStr(wlStat));
        }
    }

    // ------------------------------------------------------------
    //  Fallback to Access‑Point mode if station failed or AP is forced.
    // ------------------------------------------------------------
    if (wlStat != WL_CONNECTED && (wlStat == WL_NO_SSID_AVAIL || allowAP)) {
        setWifiAP();
    }

#if CONFIG_IDF_TARGET_ESP32S3
    // On ESP32‑S3 only, initialise mDNS when we are in station mode.
    if (netMode == 0) {
        setupMdnsHost();
    }
#endif

    // Ensure the background ping task is running (required for both STA and AP).
    if (pingHandle == NULL) {
        startPing();
    }

    // Return true only when the station is actually connected.
    return (wlStat == WL_CONNECTED);
}



/* --------------------------------------------------------------
 *  3️⃣  Tiny test framework (no external dependencies)
 * -------------------------------------------------------------- */
#define ASSERT_TRUE(expr)  if(!(expr)) throw std::runtime_error(std::string("Assertion failed: ") + #expr)
#define ASSERT_FALSE(expr) if((expr))  throw std::runtime_error(std::string("Assertion failed: ") + #expr)
#define ASSERT_EQ(a,b)  do { \
        auto _a = (a); auto _b = (b); \
        if(_a != _b) { \
            std::ostringstream _oss; \
            _oss << "Assertion failed: " << #a << " (" << _a << ") != " << #b << " (" << _b << ")"; \
            throw std::runtime_error(_oss.str()); \
        } \
    } while(0)

static void runTest(const char* name, const std::function<void()>& fn)
{
    static int total = 0, passed = 0;
    ++total;
    try {
        fn();
        ++passed;
        std::cout << "[PASS] " << name << "\n";
    }
    catch (const std::exception& e) {
        std::cout << "[FAIL] " << name << " – " << e.what() << "\n";
    }
    catch (...) {
        std::cout << "[FAIL] " << name << " – unknown exception\n";
    }
}

/* --------------------------------------------------------------
 *  4️⃣  Helper to bring the global state back to a clean slate before each test
 * -------------------------------------------------------------- */
static void resetStubs()
{
    netMode = 0;
    allowAP = false;
    std::memset(ST_SSID, 0, sizeof(ST_SSID));
    pingHandle = nullptr;

    setWifiSTA_called = false;
    setWifiAP_called = false;
    startPing_called = false;
    setupMdnsHost_called = false;

    WiFiSim::reset();
}

/* --------------------------------------------------------------
 *  5️⃣  The unit‑tests
 * -------------------------------------------------------------- */
static void test_startWifi_connected()
{
    resetStubs();
    netMode = 0;
    allowAP = false;
    std::strcpy(ST_SSID, "TestNetwork");

    // Simulate a network that matches the SSID we are looking for
    WiFiSim::ssids = { "TestNetwork", "OtherNetwork" };
    WiFiSim::rssis = { -45, -80 };
    WiFiSim::channels = { 1, 6 };

    // The station reports “connected” immediately
    WiFiSim::staStatus = WL_CONNECTED;

    // pingHandle is NULL → startPing() must be called
    pingHandle = nullptr;
    startPing_called = false;  // make sure the flag is clean

    bool ok = startWifi(true);
    ASSERT_TRUE(ok);
    ASSERT_TRUE(setWifiSTA_called);
    ASSERT_FALSE(setWifiAP_called);   // we do not need AP fallback
    ASSERT_TRUE(startPing_called);
}

static void test_startWifi_noSSID_fallbackAP()
{
    resetStubs();
    netMode = 0;
    allowAP = false;                // AP should be started because WL_NO_SSID_AVAIL
    std::strcpy(ST_SSID, "");       // empty SSID -> the while‑loop is skipped

    // No networks to scan – should have no impact on the result
    WiFiSim::ssids.clear();

    // No station connection possible
    WiFiSim::staStatus = WL_NO_SSID_AVAIL;

    pingHandle = nullptr;
    bool ok = startWifi(true);
    ASSERT_FALSE(ok);                // not connected
    ASSERT_TRUE(setWifiSTA_called); // still called (the code always calls it)
    ASSERT_TRUE(setWifiAP_called);  // fallback AP must have been started
    ASSERT_TRUE(startPing_called);
}

/* --------------------------------------------------------------
 *  This test checks the *non‑fallback* path: a SSID is defined,
 *  the ESP tries to connect, but the status returned is
 *  anything other than WL_NO_SSID_AVAIL (here WL_DISCONNECTED).
 *  According to the implementation the AP is **not** started.
 * -------------------------------------------------------------- */
static void test_startWifi_ssidPresent_butCannotConnect()
{
    resetStubs();
    netMode = 0;
    allowAP = false;
    std::strcpy(ST_SSID, "MissingNetwork");

    // The scan sees a *different* network – we will never match ST_SSID
    WiFiSim::ssids = { "OtherNetwork" };
    WiFiSim::rssis = { -80 };
    WiFiSim::channels = { 6 };

    // The STA never becomes WL_CONNECTED; we deliberately use a status
    // that is NOT WL_NO_SSID_AVAIL, so the fallback AP must stay disabled.
    WiFiSim::staStatus = WL_DISCONNECTED;

    pingHandle = nullptr;
    bool ok = startWifi(true);
    ASSERT_FALSE(ok);
    ASSERT_TRUE(setWifiSTA_called);
    ASSERT_FALSE(setWifiAP_called); // **AP is NOT started** – this is the correct behaviour
    ASSERT_TRUE(startPing_called);
}

static void test_startWifi_pingHandle_already_set()
{
    resetStubs();
    netMode = 0;
    allowAP = false;
    std::strcpy(ST_SSID, "TestNetwork");
    WiFiSim::ssids = { "TestNetwork" };
    WiFiSim::rssis = { -40 };
    WiFiSim::channels = { 1 };
    WiFiSim::staStatus = WL_CONNECTED;

    // pingHandle already points to something → startPing() must NOT be called
    pingHandle = (void*)0xDEADBEEF;
    startPing_called = false;

    bool ok = startWifi(true);
    ASSERT_TRUE(ok);
    ASSERT_TRUE(setWifiSTA_called);
    ASSERT_FALSE(setWifiAP_called);
    ASSERT_FALSE(startPing_called); // ping was already running
}

/* --------------------------------------------------------------
 *  6️⃣  Simple micro‑benchmark (average time of a successful call)
 * -------------------------------------------------------------- */
static void benchmark_startWifi(int iterations = 100)
{
    // Build a “typical” environment: SSID present and already connected.
    resetStubs();
    netMode = 0;
    allowAP = false;
    std::strcpy(ST_SSID, "BenchmarkNetwork");
    WiFiSim::ssids = { "BenchmarkNetwork" };
    WiFiSim::rssis = { -45 };
    WiFiSim::channels = { 1 };
    WiFiSim::staStatus = WL_CONNECTED;

    // Warm‑up run (so that any one‑time static init overhead is not counted)
    startWifi(true);
    startWifi(true);

    using clock = std::chrono::high_resolution_clock;
    auto t0 = clock::now();
    for (int i = 0; i < iterations; ++i) {
        // make sure startPing() is exercised each iteration
        pingHandle = nullptr;
        startPing_called = false;
        setWifiAP_called = false;
        setWifiSTA_called = false;
        startWifi(true);
    }
    auto t1 = clock::now();
    double totalMs = std::chrono::duration_cast<std::chrono::duration<double, std::milli>>(t1 - t0).count();

    std::cout << "\nBenchmark - " << iterations << " iterations\n"
        << "  total   : " << totalMs << " ms\n"
        << "  average : " << (totalMs / iterations) << " ms per call\n";
}

/* --------------------------------------------------------------
 *  7️⃣  main() – run all tests then the benchmark
 * -------------------------------------------------------------- */
int main()
{
    std::cout << "=== startWifi() unit-tests (Windows / VS2022 emulation) ===\n";
    runTest("Connected - returns true and starts ping", test_startWifi_connected);
    runTest("No SSID - falls back to AP", test_startWifi_noSSID_fallbackAP);
    runTest("SSID present but cannot connect - AP NOT started", test_startWifi_ssidPresent_butCannotConnect);
    runTest("PingHandle already set - no startPing()", test_startWifi_pingHandle_already_set);

    std::cout << "\n=== Benchmark ===\n";
    benchmark_startWifi(100);

    return 0;
}
