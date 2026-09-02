#include <Arduino.h>
#include <BLEDevice.h>
#include <FS.h>
#include <M5Cardputer.h>
#include <Preferences.h>
#include <SD.h>
#include <SPI.h>
#include <esp_gap_ble_api.h>

#include "bmi270.h"

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <cstring>
#include <string>

namespace {

constexpr char kFirmwareVersion[] = "1.4.0";
constexpr char kOrientation[] = "Zxy";
constexpr uint32_t kCameraPreRollMs = 1000;
constexpr uint32_t kCameraPostRollMs = 1000;

constexpr uint16_t rgb565(uint8_t red, uint8_t green, uint8_t blue) {
    return static_cast<uint16_t>(((red & 0xF8U) << 8U) |
                                 ((green & 0xFCU) << 3U) |
                                 (blue >> 3U));
}

constexpr uint16_t kUiBackground = rgb565(8, 10, 10);
constexpr uint16_t kUiPanel = rgb565(20, 23, 22);
constexpr uint16_t kUiPanelRaised = rgb565(27, 30, 29);
constexpr uint16_t kUiRule = rgb565(70, 76, 72);
constexpr uint16_t kUiText = rgb565(236, 232, 218);
constexpr uint16_t kUiMuted = rgb565(158, 162, 152);
constexpr uint16_t kUiCopper = rgb565(194, 129, 76);
constexpr uint16_t kUiGood = rgb565(101, 135, 108);
constexpr uint16_t kUiWarn = rgb565(177, 135, 70);
constexpr uint16_t kUiFault = rgb565(157, 69, 60);
constexpr uint16_t kUiAxisX = rgb565(154, 96, 72);
constexpr uint16_t kUiAxisY = rgb565(104, 126, 119);
constexpr uint16_t kUiAxisZ = rgb565(139, 124, 83);

constexpr uint8_t kBmiAddress = 0x69;
constexpr uint32_t kBmiI2cFrequency = 1000000;
constexpr uint32_t kSampleRateHz = 800;
constexpr uint32_t kSensorTicksPerSample = 32;
constexpr uint32_t kSensorTimeMask = 0x00FFFFFFUL;
static_assert((kSensorTicksPerSample * 625UL) / 16UL == 1250UL,
              "BMI270 800 Hz timestamp step must be 1250 us");

constexpr int kSdSck = 40;
constexpr int kSdMiso = 39;
constexpr int kSdMosi = 14;
constexpr int kSdCs = 12;

constexpr uint16_t kFifoCombinedFrameBytes = 13;
constexpr uint16_t kFifoReadThresholdBytes = 16 * kFifoCombinedFrameBytes;
constexpr uint16_t kFifoSensorTimeOverreadBytes = 250;
constexpr uint16_t kFifoHardwareBytes = 2048;
constexpr uint16_t kFifoBufferBytes =
    kFifoHardwareBytes + kFifoSensorTimeOverreadBytes + 8;
constexpr uint16_t kFifoMaxFrames = 176;

constexpr uint32_t kCalibrationSamples = kSampleRateHz * 3;
constexpr double kCalibrationMaxStdRaw = 45.0;
constexpr UBaseType_t kSampleQueueLength = 4096;

enum class SamplerState : uint8_t {
    kStarting,
    kCalibrationRequest,
    kCalibrating,
    kIdle,
    kStartRequest,
    kRecording,
    kStopRequest,
    kError,
};

enum class UiPage : uint8_t {
    kMain,
    kDiagnostics,
};

enum class CameraState : uint8_t {
    kInitializing,
    kUnpaired,
    kDisconnected,
    kConnecting,
    kReady,
    kPairing,
    kSending,
    kFault,
};

enum class CameraCommandType : uint8_t {
    kConnect,
    kPair,
    kTrigger,
};

enum class RecordFlowState : uint8_t {
    kIdle,
    kOpening,
    kPreRoll,
    kCameraStart,
    kActive,
    kCameraStop,
    kPostRoll,
    kClosing,
};

struct BusContext {
    uint8_t address;
    uint32_t frequency;
};

struct LogSample {
    uint64_t timestampUs;
    int32_t gx;
    int32_t gy;
    int32_t gz;
    int16_t ax;
    int16_t ay;
    int16_t az;
};

struct CameraCommand {
    CameraCommandType type;
    uint32_t id;
};

struct CalibrationAccumulator {
    int64_t sum[3] = {0, 0, 0};
    int64_t sumSquares[3] = {0, 0, 0};
    uint32_t count = 0;

    void reset() {
        std::memset(sum, 0, sizeof(sum));
        std::memset(sumSquares, 0, sizeof(sumSquares));
        count = 0;
    }

    void add(int16_t x, int16_t y, int16_t z) {
        const int32_t value[3] = {x, y, z};
        for (size_t axis = 0; axis < 3; ++axis) {
            sum[axis] += value[axis];
            sumSquares[axis] += static_cast<int64_t>(value[axis]) * value[axis];
        }
        ++count;
    }
};

struct SensorClock {
    bool valid = false;
    uint32_t previousRaw = 0;
    uint64_t extendedTicks = 0;

    void reset() {
        valid = false;
        previousRaw = 0;
        extendedTicks = 0;
    }

    bool extend(uint32_t raw, uint64_t* result) {
        raw &= kSensorTimeMask;
        if (!valid) {
            valid = true;
            previousRaw = raw;
            extendedTicks = raw;
            *result = extendedTicks;
            return true;
        }

        const uint32_t delta = (raw - previousRaw) & kSensorTimeMask;
        if (delta > 0x007FFFFFUL) {
            return false;
        }
        extendedTicks += delta;
        previousRaw = raw;
        *result = extendedTicks;
        return true;
    }
};

bmi2_dev gBmi{};
BusContext gBus{kBmiAddress, kBmiI2cFrequency};
SemaphoreHandle_t gI2cMutex = nullptr;
QueueHandle_t gSampleQueue = nullptr;
TaskHandle_t gSamplerTask = nullptr;

volatile SamplerState gSamplerState = SamplerState::kStarting;
volatile int8_t gSensorError = BMI2_OK;
volatile bool gConfigVerified = false;
volatile bool gCalibrationDone = false;
volatile uint32_t gCalibrationCount = 0;
volatile uint32_t gCalibrationRetries = 0;
volatile uint16_t gCalibrationStdRaw = 0;

volatile uint32_t gCapturedSamples = 0;
volatile uint32_t gDroppedQueueSamples = 0;
volatile uint32_t gMissedSensorSamples = 0;
volatile uint32_t gSkippedFifoSamples = 0;
volatile uint32_t gFifoWarnings = 0;
volatile uint32_t gClockCorrections = 0;
volatile uint32_t gSensorTimeFrames = 0;
volatile uint32_t gSensorTimeFallbacks = 0;
volatile uint32_t gSensorTimeErrors = 0;
volatile uint16_t gFifoCurrentBytes = 0;
volatile uint16_t gFifoHighWaterBytes = 0;
volatile uint16_t gRateTimes10 = 0;
volatile uint32_t gRecordingElapsedMs = 0;
volatile int16_t gUiGyroRaw[3] = {0, 0, 0};
volatile uint32_t gUiMotionSequence = 0;

int32_t gGyroBias[3] = {0, 0, 0};
CalibrationAccumulator gCalibration;
SensorClock gSensorClock;
bool gHaveLastSampleTick = false;
uint64_t gLastSampleTick = 0;
uint64_t gRecordingStartTick = 0;

uint8_t gFifoRaw[kFifoBufferBytes]{};
bmi2_sens_axes_data gFifoAccel[kFifoMaxFrames]{};
bmi2_sens_axes_data gFifoGyro[kFifoMaxFrames]{};
bmi2_fifo_frame gFifoFrame{};

File gLogFile;
char gSessionInput[13] = "SESSION";
size_t gSessionInputLength = 7;
char gSessionName[24]{};
char gSessionPath[48]{};
char gLogPath[96]{};
char gLastLogPath[96]{};
uint32_t gCurrentTake = 0;
uint32_t gWrittenSamples = 0;
uint32_t gLastFileFlushMs = 0;
uint32_t gLastFileBytes = 0;
bool gSdReady = false;
bool gWriteFailed = false;
bool gLastWriteFailed = false;
bool gSessionConfirmed = false;
bool gSessionError = false;
UiPage gUiPage = UiPage::kMain;

M5Canvas gUiCanvas(&M5Cardputer.Display);
lgfx::LovyanGFX* gUiDisplay = &M5Cardputer.Display;
bool gUiCanvasReady = false;

lgfx::LovyanGFX& uiDisplay() {
    return *gUiDisplay;
}

constexpr size_t kUiGraphSamples = 48;
int16_t gUiGraph[3][kUiGraphSamples]{};
size_t gUiGraphHead = 0;
size_t gUiGraphCount = 0;
uint32_t gUiLastMotionSequence = 0;

enum class ChargeState : uint8_t {
    kUnknown,
    kDischarging,
    kCharging,
};

int32_t gBatteryLevel = -1;
int16_t gBatteryMillivolts = -1;
ChargeState gChargeState = ChargeState::kUnknown;
bool gChargeStateEstimated = false;
uint32_t gLastPowerReadMs = 0;
constexpr size_t kPowerHistorySamples = 12;
int16_t gPowerHistory[kPowerHistorySamples]{};
size_t gPowerHistoryHead = 0;
size_t gPowerHistoryCount = 0;

class CanonSecurityCallbacks : public BLESecurityCallbacks {
public:
    uint32_t onPassKeyRequest() override {
        return 123456;
    }

    void onPassKeyNotify(uint32_t) override {
    }

    bool onConfirmPIN(uint32_t) override {
        return true;
    }

    bool onSecurityRequest() override {
        return true;
    }

    void onAuthenticationComplete(esp_ble_auth_cmpl_t) override {
    }
};

class CanonConnectionCallbacks : public BLEClientCallbacks {
public:
    explicit CanonConnectionCallbacks(volatile bool* connected)
        : connected_(connected) {
    }

    void onConnect(BLEClient*) override {
        *connected_ = true;
    }

    void onDisconnect(BLEClient*) override {
        *connected_ = false;
    }

private:
    volatile bool* connected_;
};

class CanonScanCallbacks : public BLEAdvertisedDeviceCallbacks {
public:
    CanonScanCallbacks(const BLEUUID& target, bool* found, char* address, size_t addressSize)
        : target_(target), found_(found), address_(address), addressSize_(addressSize) {
    }

    void onResult(BLEAdvertisedDevice advertisedDevice) override {
        if (!advertisedDevice.haveServiceUUID() ||
            !target_.equals(advertisedDevice.getServiceUUID())) {
            return;
        }
        const std::string address = advertisedDevice.getAddress().toString();
        std::snprintf(address_, addressSize_, "%s", address.c_str());
        *found_ = true;
        BLEDevice::getScan()->stop();
    }

private:
    BLEUUID target_;
    bool* found_;
    char* address_;
    size_t addressSize_;
};

class CanonRemote {
public:
    CanonRemote()
        : serviceUuid_("00050000-0000-1000-0000-d8492fffa821"),
          pairingUuid_("00050002-0000-1000-0000-d8492fffa821"),
          triggerUuid_("00050003-0000-1000-0000-d8492fffa821"),
          connectionCallbacks_(&connected_) {
    }

    bool begin() {
        BLEDevice::init("ADV GYRO");
        BLEDevice::setEncryptionLevel(ESP_BLE_SEC_ENCRYPT_NO_MITM);
        BLEDevice::setSecurityCallbacks(new CanonSecurityCallbacks());
        client_ = BLEDevice::createClient();
        if (client_ == nullptr) {
            return false;
        }
        client_->setClientCallbacks(&connectionCallbacks_);

        Preferences preferences;
        if (preferences.begin("advgyro", true)) {
            address_ = preferences.getString("cameraaddr", "");
            preferences.end();
        }
        initialized_ = true;
        return true;
    }

    bool hasPairedCamera() const {
        return address_.length() == 17;
    }

    bool isConnected() const {
        return connected_ && client_ != nullptr && client_->isConnected();
    }

    bool connect() {
        if (!initialized_ || !hasPairedCamera() || client_ == nullptr) {
            return false;
        }
        if (isConnected() && triggerCharacteristic_ != nullptr) {
            return true;
        }

        disconnect();
        BLEAddress address(address_.c_str());
        if (!client_->connect(address)) {
            return false;
        }

        BLERemoteService* service = client_->getService(serviceUuid_);
        if (service == nullptr) {
            disconnect();
            return false;
        }

        triggerCharacteristic_ = service->getCharacteristic(triggerUuid_);
        if (triggerCharacteristic_ == nullptr) {
            disconnect();
            return false;
        }
        return true;
    }

    bool pair(uint32_t scanSeconds) {
        if (!initialized_ || client_ == nullptr) {
            return false;
        }

        disconnect();
        BLEDevice::setEncryptionLevel(ESP_BLE_SEC_ENCRYPT_MITM);

        bool found = false;
        char foundAddress[18]{};
        CanonScanCallbacks callbacks(serviceUuid_, &found, foundAddress, sizeof(foundAddress));
        BLEScan* scan = BLEDevice::getScan();
        scan->setAdvertisedDeviceCallbacks(&callbacks);
        scan->setActiveScan(true);
        scan->start(scanSeconds, false);
        scan->stop();
        scan->clearResults();

        if (!found || std::strlen(foundAddress) != 17) {
            BLEDevice::setEncryptionLevel(ESP_BLE_SEC_ENCRYPT_NO_MITM);
            return false;
        }

        address_ = foundAddress;
        BLEAddress address(address_.c_str());
        if (!client_->connect(address)) {
            BLEDevice::setEncryptionLevel(ESP_BLE_SEC_ENCRYPT_NO_MITM);
            return false;
        }

        BLERemoteService* service = client_->getService(serviceUuid_);
        if (service == nullptr) {
            disconnect();
            BLEDevice::setEncryptionLevel(ESP_BLE_SEC_ENCRYPT_NO_MITM);
            return false;
        }

        BLERemoteCharacteristic* pairing = service->getCharacteristic(pairingUuid_);
        if (pairing == nullptr) {
            disconnect();
            BLEDevice::setEncryptionLevel(ESP_BLE_SEC_ENCRYPT_NO_MITM);
            return false;
        }

        const char deviceName[] = " ADV GYRO ";
        uint8_t payload[sizeof(deviceName) - 1]{};
        std::memcpy(payload, deviceName, sizeof(payload));
        payload[0] = 0x03;
        pairing->writeValue(payload, sizeof(payload), false);
        delay(250);
        disconnect();
        BLEDevice::setEncryptionLevel(ESP_BLE_SEC_ENCRYPT_NO_MITM);

        Preferences preferences;
        bool saved = false;
        if (preferences.begin("advgyro", false)) {
            saved = preferences.putString("cameraaddr", address_) == address_.length();
            preferences.end();
        }
        if (!saved) {
            return false;
        }

        delay(250);
        return connect();
    }

    bool trigger() {
        if (!connect() || triggerCharacteristic_ == nullptr) {
            return false;
        }

        uint8_t pressed = 0x8C;
        uint8_t released = 0x0C;
        triggerCharacteristic_->writeValue(&pressed, 1, false);
        delay(200);
        triggerCharacteristic_->writeValue(&released, 1, false);
        delay(50);
        return true;
    }

private:
    void disconnect() {
        triggerCharacteristic_ = nullptr;
        if (client_ != nullptr && client_->isConnected()) {
            client_->disconnect();
        }
        connected_ = false;
    }

    BLEUUID serviceUuid_;
    BLEUUID pairingUuid_;
    BLEUUID triggerUuid_;
    BLEClient* client_ = nullptr;
    BLERemoteCharacteristic* triggerCharacteristic_ = nullptr;
    volatile bool connected_ = false;
    CanonConnectionCallbacks connectionCallbacks_;
    String address_;
    bool initialized_ = false;
};

CanonRemote gCanonRemote;
QueueHandle_t gCameraQueue = nullptr;
TaskHandle_t gCameraTask = nullptr;
volatile CameraState gCameraState = CameraState::kInitializing;
volatile uint32_t gCameraCompletedCommandId = 0;
volatile bool gCameraLastCommandOk = false;
uint32_t gNextCameraCommandId = 1;

RecordFlowState gRecordFlowState = RecordFlowState::kIdle;
uint32_t gRecordFlowTimerMs = 0;
uint32_t gPendingCameraCommandId = 0;
bool gCameraRecordingAssumed = false;
bool gCameraStartOk = false;
bool gCameraStopOk = false;
bool gStopRequested = false;

void updatePowerTelemetry(bool force = false) {
    const uint32_t now = millis();
    if (!force && now - gLastPowerReadMs < 2000) {
        return;
    }
    gLastPowerReadMs = now;

    const int16_t measuredMv = M5Cardputer.Power.getBatteryVoltage();
    const int32_t measuredLevel = M5Cardputer.Power.getBatteryLevel();
    if (measuredMv > 2500 && measuredMv < 5000) {
        if (gBatteryMillivolts < 0) {
            gBatteryMillivolts = measuredMv;
        } else {
            gBatteryMillivolts = static_cast<int16_t>(
                (static_cast<int32_t>(gBatteryMillivolts) * 3 + measuredMv) / 4);
        }
    }
    if (measuredLevel >= 0 && measuredLevel <= 100) {
        if (gBatteryLevel < 0) {
            gBatteryLevel = measuredLevel;
        } else {
            gBatteryLevel = (gBatteryLevel * 3 + measuredLevel + 2) / 4;
        }
    }

    const auto hardwareCharge = M5Cardputer.Power.isCharging();
    if (hardwareCharge == m5::Power_Class::is_charging) {
        gChargeState = ChargeState::kCharging;
        gChargeStateEstimated = false;
        return;
    }
    if (hardwareCharge == m5::Power_Class::is_discharging) {
        gChargeState = ChargeState::kDischarging;
        gChargeStateEstimated = false;
        return;
    }

    if (gBatteryMillivolts > 0) {
        gPowerHistory[gPowerHistoryHead] = gBatteryMillivolts;
        gPowerHistoryHead = (gPowerHistoryHead + 1) % kPowerHistorySamples;
        gPowerHistoryCount = std::min(gPowerHistoryCount + 1, kPowerHistorySamples);
    }
    if (gPowerHistoryCount == kPowerHistorySamples) {
        int32_t oldAverage = 0;
        int32_t newAverage = 0;
        for (size_t index = 0; index < 3; ++index) {
            oldAverage += gPowerHistory[(gPowerHistoryHead + index) % kPowerHistorySamples];
            newAverage += gPowerHistory[(gPowerHistoryHead + kPowerHistorySamples - 3 + index) %
                                        kPowerHistorySamples];
        }
        const int32_t riseMv = (newAverage - oldAverage) / 3;
        if (riseMv >= 8 && gBatteryLevel < 100) {
            gChargeState = ChargeState::kCharging;
            gChargeStateEstimated = true;
        } else if (riseMv <= -6) {
            gChargeState = ChargeState::kDischarging;
            gChargeStateEstimated = true;
        }
    }
}

BMI2_INTF_RETURN_TYPE bmiRead(uint8_t regAddr, uint8_t* data, uint32_t length, void* intfPtr) {
    const auto* bus = static_cast<const BusContext*>(intfPtr);
    if (bus == nullptr || data == nullptr || gI2cMutex == nullptr) {
        return -1;
    }

    if (xSemaphoreTake(gI2cMutex, pdMS_TO_TICKS(100)) != pdTRUE) {
        return -1;
    }
    const bool ok = M5Cardputer.In_I2C.readRegister(
        bus->address, regAddr, data, static_cast<size_t>(length), bus->frequency);
    xSemaphoreGive(gI2cMutex);
    return ok ? BMI2_INTF_RET_SUCCESS : -1;
}

BMI2_INTF_RETURN_TYPE bmiWrite(
    uint8_t regAddr, const uint8_t* data, uint32_t length, void* intfPtr) {
    const auto* bus = static_cast<const BusContext*>(intfPtr);
    if (bus == nullptr || data == nullptr || gI2cMutex == nullptr) {
        return -1;
    }

    if (xSemaphoreTake(gI2cMutex, pdMS_TO_TICKS(100)) != pdTRUE) {
        return -1;
    }
    const bool ok = M5Cardputer.In_I2C.writeRegister(
        bus->address, regAddr, data, static_cast<size_t>(length), bus->frequency);
    xSemaphoreGive(gI2cMutex);
    return ok ? BMI2_INTF_RET_SUCCESS : -1;
}

void bmiDelayUs(uint32_t periodUs, void*) {
    if (periodUs >= 1000) {
        delay((periodUs + 999) / 1000);
    } else {
        delayMicroseconds(periodUs);
    }
}

bool checkBmi(int8_t result) {
    if (result == BMI2_OK) {
        return true;
    }
    gSensorError = result;
    gSamplerState = SamplerState::kError;
    return false;
}

bool configureBmi270() {
    std::memset(&gBmi, 0, sizeof(gBmi));
    gBmi.intf = BMI2_I2C_INTF;
    gBmi.intf_ptr = &gBus;
    gBmi.read = bmiRead;
    gBmi.write = bmiWrite;
    gBmi.delay_us = bmiDelayUs;
    gBmi.read_write_len = 32;

    if (!checkBmi(bmi270_init(&gBmi))) {
        return false;
    }
    if (gBmi.chip_id != BMI270_CHIP_ID) {
        gSensorError = BMI2_E_DEV_NOT_FOUND;
        gSamplerState = SamplerState::kError;
        return false;
    }
    if (!checkBmi(bmi2_set_adv_power_save(BMI2_DISABLE, &gBmi))) {
        return false;
    }

    constexpr uint16_t allFifoOptions = BMI2_FIFO_ALL_EN | BMI2_FIFO_HEADER_EN |
                                         BMI2_FIFO_TIME_EN | BMI2_FIFO_STOP_ON_FULL |
                                         BMI2_FIFO_TAG_INT1 | BMI2_FIFO_TAG_INT2;
    if (!checkBmi(bmi2_set_fifo_config(allFifoOptions, BMI2_DISABLE, &gBmi))) {
        return false;
    }

    bmi2_sens_config config[2]{};
    config[0].type = BMI2_ACCEL;
    config[1].type = BMI2_GYRO;
    if (!checkBmi(bmi270_get_sensor_config(config, 2, &gBmi))) {
        return false;
    }

    config[0].cfg.acc.odr = BMI2_ACC_ODR_800HZ;
    config[0].cfg.acc.range = BMI2_ACC_RANGE_16G;
    config[0].cfg.acc.bwp = BMI2_ACC_OSR2_AVG2;
    config[0].cfg.acc.filter_perf = BMI2_PERF_OPT_MODE;

    config[1].cfg.gyr.odr = BMI2_GYR_ODR_800HZ;
    config[1].cfg.gyr.range = BMI2_GYR_RANGE_2000;
    config[1].cfg.gyr.bwp = BMI2_GYR_OSR4_MODE;
    config[1].cfg.gyr.noise_perf = BMI2_PERF_OPT_MODE;
    config[1].cfg.gyr.filter_perf = BMI2_PERF_OPT_MODE;

    if (!checkBmi(bmi270_set_sensor_config(config, 2, &gBmi))) {
        return false;
    }

    uint8_t sensors[2] = {BMI2_ACCEL, BMI2_GYRO};
    if (!checkBmi(bmi270_sensor_enable(sensors, 2, &gBmi))) {
        return false;
    }
    delay(50);

    if (!checkBmi(bmi2_set_fifo_filter_data(BMI2_ACCEL, BMI2_FIFO_FILTERED_DATA, &gBmi)) ||
        !checkBmi(bmi2_set_fifo_filter_data(BMI2_GYRO, BMI2_FIFO_FILTERED_DATA, &gBmi)) ||
        !checkBmi(bmi2_set_fifo_down_sample(BMI2_ACCEL, 0, &gBmi)) ||
        !checkBmi(bmi2_set_fifo_down_sample(BMI2_GYRO, 0, &gBmi))) {
        return false;
    }

    constexpr uint16_t enabledFifoOptions = BMI2_FIFO_ACC_EN | BMI2_FIFO_GYR_EN |
                                             BMI2_FIFO_HEADER_EN | BMI2_FIFO_TIME_EN |
                                             BMI2_FIFO_STOP_ON_FULL;
    if (!checkBmi(bmi2_set_fifo_config(enabledFifoOptions, BMI2_ENABLE, &gBmi))) {
        return false;
    }
    if (!checkBmi(bmi2_set_command_register(BMI2_FIFO_FLUSH_CMD, &gBmi))) {
        return false;
    }

    bmi2_sens_config verify[2]{};
    verify[0].type = BMI2_ACCEL;
    verify[1].type = BMI2_GYRO;
    uint16_t fifoConfig = 0;
    if (!checkBmi(bmi270_get_sensor_config(verify, 2, &gBmi)) ||
        !checkBmi(bmi2_get_fifo_config(&fifoConfig, &gBmi))) {
        return false;
    }

    const uint16_t requiredFifo = BMI2_FIFO_ACC_EN | BMI2_FIFO_GYR_EN |
                                  BMI2_FIFO_HEADER_EN | BMI2_FIFO_TIME_EN;
    gConfigVerified = verify[0].cfg.acc.odr == BMI2_ACC_ODR_800HZ &&
                      verify[1].cfg.gyr.odr == BMI2_GYR_ODR_800HZ &&
                      verify[1].cfg.gyr.range == BMI2_GYR_RANGE_2000 &&
                      verify[1].cfg.gyr.bwp == BMI2_GYR_OSR4_MODE &&
                      (fifoConfig & requiredFifo) == requiredFifo;
    if (!gConfigVerified) {
        gSensorError = BMI2_E_INVALID_SENSOR;
        gSamplerState = SamplerState::kError;
        return false;
    }

    gFifoFrame.data = gFifoRaw;
    gFifoFrame.length = kFifoBufferBytes;
    return true;
}

bool flushSensorFifo() {
    return checkBmi(bmi2_set_command_register(BMI2_FIFO_FLUSH_CMD, &gBmi));
}

bool readDirectSensorTime(uint32_t* sensorTime) {
    uint8_t raw[3]{};
    const int8_t result = bmi2_get_regs(BMI2_SENSORTIME_ADDR, raw, sizeof(raw), &gBmi);
    if (result != BMI2_OK) {
        return false;
    }
    *sensorTime = static_cast<uint32_t>(raw[0]) |
                  (static_cast<uint32_t>(raw[1]) << 8) |
                  (static_cast<uint32_t>(raw[2]) << 16);
    return true;
}

void processCalibrationSamples(uint16_t count) {
    for (uint16_t i = 0; i < count && gCalibration.count < kCalibrationSamples; ++i) {
        gCalibration.add(gFifoGyro[i].x, gFifoGyro[i].y, gFifoGyro[i].z);
    }
    gCalibrationCount = gCalibration.count;

    if (gCalibration.count < kCalibrationSamples) {
        return;
    }

    double maxStd = 0.0;
    for (size_t axis = 0; axis < 3; ++axis) {
        const double mean = static_cast<double>(gCalibration.sum[axis]) / gCalibration.count;
        const double meanSquares =
            static_cast<double>(gCalibration.sumSquares[axis]) / gCalibration.count;
        const double variance = std::max(0.0, meanSquares - mean * mean);
        maxStd = std::max(maxStd, std::sqrt(variance));
    }
    gCalibrationStdRaw = static_cast<uint16_t>(std::lround(maxStd));

    if (maxStd > kCalibrationMaxStdRaw) {
        ++gCalibrationRetries;
        gCalibration.reset();
        gCalibrationCount = 0;
        return;
    }

    for (size_t axis = 0; axis < 3; ++axis) {
        gGyroBias[axis] = static_cast<int32_t>(std::lround(
            static_cast<double>(gCalibration.sum[axis]) / gCalibration.count));
    }
    gCalibrationDone = true;
    gSamplerState = SamplerState::kIdle;
}

void resetRecordingStats() {
    gCapturedSamples = 0;
    gDroppedQueueSamples = 0;
    gMissedSensorSamples = 0;
    gSkippedFifoSamples = 0;
    gFifoWarnings = 0;
    gClockCorrections = 0;
    gSensorTimeFrames = 0;
    gSensorTimeFallbacks = 0;
    gSensorTimeErrors = 0;
    gFifoCurrentBytes = 0;
    gFifoHighWaterBytes = 0;
    gRateTimes10 = 0;
    gRecordingElapsedMs = 0;
    gSensorClock.reset();
    gHaveLastSampleTick = false;
    gLastSampleTick = 0;
    gRecordingStartTick = 0;
}

void queueRecordingSamples(
    uint16_t count, uint32_t rawSensorTime, bool haveSensorTime, uint8_t skippedFrames) {
    if (count == 0) {
        return;
    }

    uint64_t extendedSensorTime = 0;
    if (!haveSensorTime || !gSensorClock.extend(rawSensorTime, &extendedSensorTime)) {
        ++gSensorTimeErrors;
        if (!gHaveLastSampleTick) {
            gMissedSensorSamples += count;
            return;
        }
        extendedSensorTime = gLastSampleTick +
                             static_cast<uint64_t>(count + skippedFrames) *
                                 kSensorTicksPerSample;
    }

    uint64_t lastTick = extendedSensorTime &
                        ~static_cast<uint64_t>(kSensorTicksPerSample - 1);
    const uint64_t batchSpan = static_cast<uint64_t>(count - 1) * kSensorTicksPerSample;
    uint64_t firstTick = lastTick >= batchSpan ? lastTick - batchSpan : 0;

    uint32_t gapFrames = 0;
    if (!gHaveLastSampleTick) {
        gRecordingStartTick = firstTick;
    } else {
        const uint64_t expectedFirst = gLastSampleTick + kSensorTicksPerSample;
        if (firstTick > expectedFirst) {
            gapFrames = static_cast<uint32_t>((firstTick - expectedFirst) /
                                              kSensorTicksPerSample);
        } else if (firstTick < expectedFirst) {
            firstTick = expectedFirst;
            lastTick = firstTick + batchSpan;
            ++gClockCorrections;
        }
    }

    gSkippedFifoSamples += skippedFrames;
    gMissedSensorSamples += std::max<uint32_t>(gapFrames, skippedFrames);

    for (uint16_t i = 0; i < count; ++i) {
        const uint64_t tick = firstTick + static_cast<uint64_t>(i) * kSensorTicksPerSample;
        const uint64_t relativeTicks = tick - gRecordingStartTick;

        LogSample sample{};
        sample.timestampUs = (relativeTicks * 625ULL) / 16ULL;
        sample.gx = static_cast<int32_t>(gFifoGyro[i].x) - gGyroBias[0];
        sample.gy = static_cast<int32_t>(gFifoGyro[i].y) - gGyroBias[1];
        sample.gz = static_cast<int32_t>(gFifoGyro[i].z) - gGyroBias[2];
        sample.ax = gFifoAccel[i].x;
        sample.ay = gFifoAccel[i].y;
        sample.az = gFifoAccel[i].z;

        ++gCapturedSamples;
        if (xQueueSend(gSampleQueue, &sample, 0) != pdTRUE) {
            ++gDroppedQueueSamples;
        }
    }

    gHaveLastSampleTick = true;
    gLastSampleTick = lastTick;
    const uint64_t elapsedUs =
        ((gLastSampleTick - gRecordingStartTick) * 625ULL) / 16ULL;
    gRecordingElapsedMs = static_cast<uint32_t>(elapsedUs / 1000ULL);
    if (elapsedUs > 0 && gCapturedSamples > 1) {
        const uint64_t rateTimes10 =
            (static_cast<uint64_t>(gCapturedSamples - 1) * 10000000ULL) / elapsedUs;
        gRateTimes10 = static_cast<uint16_t>(std::min<uint64_t>(rateTimes10, 9999));
    }
}

void publishUiMotion(uint16_t frameCount) {
    if (frameCount == 0) {
        return;
    }
    const size_t index = frameCount - 1;
    const int32_t values[3] = {
        static_cast<int32_t>(gFifoGyro[index].x) - gGyroBias[0],
        static_cast<int32_t>(gFifoGyro[index].y) - gGyroBias[1],
        static_cast<int32_t>(gFifoGyro[index].z) - gGyroBias[2],
    };
    for (size_t axis = 0; axis < 3; ++axis) {
        const int32_t clamped = std::max<int32_t>(
            INT16_MIN, std::min<int32_t>(INT16_MAX, values[axis]));
        gUiGyroRaw[axis] = static_cast<int16_t>(clamped);
    }
    ++gUiMotionSequence;
}

int readFifoBatch(bool forceRead, bool capture) {
    uint16_t availableBytes = 0;
    if (!checkBmi(bmi2_get_fifo_length(&availableBytes, &gBmi))) {
        return -1;
    }
    gFifoCurrentBytes = availableBytes;
    if (availableBytes > gFifoHighWaterBytes) {
        gFifoHighWaterBytes = availableBytes;
    }
    if (availableBytes >= 1800) {
        ++gFifoWarnings;
    }
    if (availableBytes == 0 || (!forceRead && availableBytes < kFifoReadThresholdBytes)) {
        return 0;
    }

    const uint16_t readLength = std::min<uint16_t>(
        static_cast<uint16_t>(availableBytes + kFifoSensorTimeOverreadBytes),
        kFifoBufferBytes);
    gFifoFrame.length = readLength;
    if (!checkBmi(bmi2_read_fifo_data(&gFifoFrame, &gBmi))) {
        return -1;
    }

    gFifoFrame.sensor_time = UINT32_MAX;
    gFifoFrame.skipped_frame_count = 0;
    uint16_t accelFrames = kFifoMaxFrames;
    uint16_t gyroFrames = kFifoMaxFrames;
    const int8_t accelResult =
        bmi2_extract_accel(gFifoAccel, &accelFrames, &gFifoFrame, &gBmi);
    const int8_t gyroResult =
        bmi2_extract_gyro(gFifoGyro, &gyroFrames, &gFifoFrame, &gBmi);
    if (accelResult < BMI2_OK || gyroResult < BMI2_OK) {
        gSensorError = accelResult < BMI2_OK ? accelResult : gyroResult;
        gSamplerState = SamplerState::kError;
        return -1;
    }

    const uint16_t pairedFrames = std::min(accelFrames, gyroFrames);
    if (accelFrames != gyroFrames) {
        ++gFifoWarnings;
    }
    publishUiMotion(pairedFrames);

    uint32_t sensorTime = gFifoFrame.sensor_time;
    bool haveSensorTime = sensorTime != UINT32_MAX;
    if (haveSensorTime) {
        if (capture) {
            ++gSensorTimeFrames;
        }
    } else if (pairedFrames > 0 && readDirectSensorTime(&sensorTime)) {
        haveSensorTime = true;
        if (capture) {
            ++gSensorTimeFallbacks;
        }
    }

    if (gSamplerState == SamplerState::kCalibrating) {
        processCalibrationSamples(pairedFrames);
    } else if (capture && gSamplerState != SamplerState::kError) {
        queueRecordingSamples(
            pairedFrames, sensorTime, haveSensorTime, gFifoFrame.skipped_frame_count);
    }
    return pairedFrames;
}

void samplerTask(void*) {
    gSamplerState = SamplerState::kCalibrationRequest;

    for (;;) {
        const SamplerState state = gSamplerState;
        if (state == SamplerState::kError) {
            vTaskDelay(pdMS_TO_TICKS(100));
            continue;
        }

        if (state == SamplerState::kCalibrationRequest) {
            gCalibration.reset();
            gCalibrationCount = 0;
            gCalibrationRetries = 0;
            gCalibrationStdRaw = 0;
            gCalibrationDone = false;
            if (!flushSensorFifo()) {
                continue;
            }
            gSamplerState = SamplerState::kCalibrating;
            continue;
        }

        if (state == SamplerState::kStartRequest) {
            if (!flushSensorFifo()) {
                continue;
            }
            xQueueReset(gSampleQueue);
            resetRecordingStats();
            gSamplerState = SamplerState::kRecording;
            continue;
        }

        if (state == SamplerState::kStopRequest) {
            readFifoBatch(true, true);
            gSamplerState = SamplerState::kIdle;
            continue;
        }

        if (state == SamplerState::kRecording) {
            readFifoBatch(false, true);
        } else {
            readFifoBatch(false, false);
        }
        vTaskDelay(pdMS_TO_TICKS(1));
    }
}

bool initializeSd() {
    SPI.begin(kSdSck, kSdMiso, kSdMosi, kSdCs);
    if (!SD.begin(kSdCs, SPI, 25000000)) {
        return false;
    }
    if (SD.cardType() == CARD_NONE) {
        return false;
    }
    if (!SD.exists("/GYRO") && !SD.mkdir("/GYRO")) {
        return false;
    }
    return true;
}

bool createSessionFolder() {
    if (!gSdReady) {
        gSdReady = initializeSd();
    }
    if (!gSdReady) {
        gSessionError = true;
        return false;
    }

    const char* base = gSessionInputLength == 0 ? "SESSION" : gSessionInput;
    for (uint32_t number = 1; number <= 999; ++number) {
        char finalName[24];
        if (number == 1) {
            std::snprintf(finalName, sizeof(finalName), "%s", base);
        } else {
            std::snprintf(finalName, sizeof(finalName), "%s_%02lu", base,
                          static_cast<unsigned long>(number));
        }

        char candidate[48];
        std::snprintf(candidate, sizeof(candidate), "/GYRO/%s", finalName);
        if (!SD.exists(candidate)) {
            if (!SD.mkdir(candidate)) {
                gSessionError = true;
                return false;
            }
            std::snprintf(gSessionName, sizeof(gSessionName), "%s", finalName);
            std::snprintf(gSessionPath, sizeof(gSessionPath), "%s", candidate);
            gSessionConfirmed = true;
            gSessionError = false;
            gCurrentTake = 0;
            gLastLogPath[0] = '\0';
            gUiPage = UiPage::kMain;
            return true;
        }
    }

    gSessionError = true;
    return false;
}

void openSessionEditor(bool clearName) {
    if (clearName) {
        gSessionInputLength = 0;
        gSessionInput[0] = '\0';
    }
    gSessionConfirmed = false;
    gSessionError = false;
    gUiPage = UiPage::kMain;
}

char sanitizeSessionChar(char value) {
    if (value >= 'a' && value <= 'z') {
        value = static_cast<char>(value - 'a' + 'A');
    }
    if ((value >= 'A' && value <= 'Z') || (value >= '0' && value <= '9')) {
        return value;
    }
    if (value == ' ' || value == '-' || value == '_') {
        return '_';
    }
    return '\0';
}

bool chooseNextLogPath() {
    if (!gSessionConfirmed || gSessionPath[0] == '\0') {
        return false;
    }

    for (uint32_t number = 1; number <= 9999; ++number) {
        std::snprintf(gLogPath, sizeof(gLogPath), "%s/%s_%03lu.gcsv",
                      gSessionPath, gSessionName, static_cast<unsigned long>(number));
        if (!SD.exists(gLogPath)) {
            gCurrentTake = number;
            return true;
        }
    }
    return false;
}

bool openLogFile() {
    if (!gSdReady || !chooseNextLogPath()) {
        return false;
    }

    gLogFile = SD.open(gLogPath, FILE_WRITE);
    if (!gLogFile) {
        return false;
    }

    gLogFile.print("GYROFLOW IMU LOG\n");
    gLogFile.print("version,1.3\n");
    gLogFile.print("id,m5stack_cardputer_adv_bmi270\n");
    gLogFile.printf("orientation,%s\n", kOrientation);
    gLogFile.print("note,BMI270 800Hz FIFO header sensortime OSR4 high-performance\n");
    gLogFile.printf("fwversion,%s\n", kFirmwareVersion);
    gLogFile.print("vendor,M5Stack community firmware\n");
    gLogFile.printf("session,%s\n", gSessionName);
    gLogFile.printf("take,%lu\n", static_cast<unsigned long>(gCurrentTake));
    gLogFile.print("tscale,0.000001000\n");
    gLogFile.print("gscale,0.0010652644360317\n");
    gLogFile.print("ascale,0.00048828125\n");
    gLogFile.print("t,gx,gy,gz,ax,ay,az\n");
    gLogFile.flush();

    gWrittenSamples = 0;
    gWriteFailed = false;
    gLastWriteFailed = false;
    gLastFileFlushMs = millis();
    return true;
}

bool writeBlock(const char* data, size_t length) {
    if (!gLogFile || length == 0) {
        return length == 0;
    }
    const size_t written =
        gLogFile.write(reinterpret_cast<const uint8_t*>(data), length);
    if (written != length) {
        gWriteFailed = true;
        return false;
    }
    return true;
}

void serviceLogWriter(size_t maxSamples = 768) {
    if (!gLogFile || gSampleQueue == nullptr || gWriteFailed) {
        return;
    }

    static char output[24576];
    size_t used = 0;
    size_t handled = 0;
    LogSample sample{};

    while (handled < maxSamples && xQueueReceive(gSampleQueue, &sample, 0) == pdTRUE) {
        char line[112];
        const int length = std::snprintf(
            line, sizeof(line), "%llu,%ld,%ld,%ld,%d,%d,%d\n",
            static_cast<unsigned long long>(sample.timestampUs),
            static_cast<long>(sample.gx), static_cast<long>(sample.gy),
            static_cast<long>(sample.gz), static_cast<int>(sample.ax),
            static_cast<int>(sample.ay), static_cast<int>(sample.az));
        if (length <= 0 || static_cast<size_t>(length) >= sizeof(line)) {
            gWriteFailed = true;
            break;
        }
        if (used + static_cast<size_t>(length) > sizeof(output)) {
            if (!writeBlock(output, used)) {
                break;
            }
            used = 0;
        }
        std::memcpy(output + used, line, static_cast<size_t>(length));
        used += static_cast<size_t>(length);
        ++handled;
        ++gWrittenSamples;
    }
    writeBlock(output, used);

    const uint32_t now = millis();
    if (now - gLastFileFlushMs >= 1000) {
        gLogFile.flush();
        gLastFileFlushMs = now;
    }
}

bool beginRecording() {
    if (gSamplerState != SamplerState::kIdle || !gCalibrationDone || !openLogFile()) {
        return false;
    }
    gUiPage = UiPage::kMain;
    gSamplerState = SamplerState::kStartRequest;
    return true;
}

void requestStopRecording() {
    if (gSamplerState == SamplerState::kRecording ||
        gSamplerState == SamplerState::kStartRequest) {
        gSamplerState = SamplerState::kStopRequest;
    }
}

void finishRecordingIfReady() {
    if (!gLogFile || gSamplerState != SamplerState::kIdle) {
        return;
    }
    while (uxQueueMessagesWaiting(gSampleQueue) > 0 && !gWriteFailed) {
        serviceLogWriter(1024);
    }
    gLogFile.flush();
    gLastFileBytes = static_cast<uint32_t>(gLogFile.size());
    gLastWriteFailed = gWriteFailed;
    std::strncpy(gLastLogPath, gLogPath, sizeof(gLastLogPath) - 1);
    gLastLogPath[sizeof(gLastLogPath) - 1] = '\0';
    gLogFile.close();
    if (gWriteFailed) {
        gSdReady = false;
    }
}

void requestCalibration() {
    if (gSamplerState == SamplerState::kIdle && !gLogFile &&
        gRecordFlowState == RecordFlowState::kIdle) {
        gUiPage = UiPage::kMain;
        gSamplerState = SamplerState::kCalibrationRequest;
    }
}

uint32_t queueCameraCommand(CameraCommandType type) {
    if (gCameraQueue == nullptr) {
        return 0;
    }
    CameraCommand command{type, gNextCameraCommandId++};
    if (xQueueSend(gCameraQueue, &command, 0) != pdTRUE) {
        return 0;
    }
    return command.id;
}

void cameraTask(void*) {
    CameraCommand command{};
    for (;;) {
        if (xQueueReceive(gCameraQueue, &command, portMAX_DELAY) != pdTRUE) {
            continue;
        }

        bool result = false;
        switch (command.type) {
            case CameraCommandType::kConnect:
                gCameraState = CameraState::kConnecting;
                result = gCanonRemote.connect();
                break;
            case CameraCommandType::kPair:
                gCameraState = CameraState::kPairing;
                result = gCanonRemote.pair(20);
                break;
            case CameraCommandType::kTrigger:
                gCameraState = CameraState::kSending;
                result = gCanonRemote.trigger();
                break;
        }

        gCameraLastCommandOk = result;
        if (result) {
            gCameraState = CameraState::kReady;
        } else if (!gCanonRemote.hasPairedCamera()) {
            gCameraState = CameraState::kUnpaired;
        } else if (command.type == CameraCommandType::kPair) {
            gCameraState = CameraState::kFault;
        } else {
            gCameraState = CameraState::kDisconnected;
        }
        gCameraCompletedCommandId = command.id;
    }
}

void beginPostRoll() {
    gRecordFlowTimerMs = millis();
    gRecordFlowState = RecordFlowState::kPostRoll;
}

void beginCameraStop() {
    if (!gCameraRecordingAssumed) {
        beginPostRoll();
        return;
    }

    gPendingCameraCommandId = queueCameraCommand(CameraCommandType::kTrigger);
    if (gPendingCameraCommandId == 0) {
        gCameraStopOk = false;
        beginPostRoll();
        return;
    }
    gRecordFlowState = RecordFlowState::kCameraStop;
}

void handleRecordControl() {
    if (!gSessionConfirmed) {
        createSessionFolder();
        return;
    }

    if (gRecordFlowState == RecordFlowState::kIdle) {
        gCameraRecordingAssumed = false;
        gCameraStartOk = false;
        gCameraStopOk = false;
        gStopRequested = false;
        if (beginRecording()) {
            gRecordFlowState = RecordFlowState::kOpening;
        }
        return;
    }

    switch (gRecordFlowState) {
        case RecordFlowState::kOpening:
        case RecordFlowState::kPreRoll:
            gStopRequested = true;
            break;
        case RecordFlowState::kCameraStart:
            gStopRequested = true;
            break;
        case RecordFlowState::kActive:
            beginCameraStop();
            break;
        default:
            break;
    }
}

void serviceRecordFlow() {
    switch (gRecordFlowState) {
        case RecordFlowState::kIdle:
            break;

        case RecordFlowState::kOpening:
            if (gSamplerState == SamplerState::kRecording) {
                if (gStopRequested) {
                    beginPostRoll();
                } else {
                    gRecordFlowTimerMs = millis();
                    gRecordFlowState = RecordFlowState::kPreRoll;
                }
            }
            break;

        case RecordFlowState::kPreRoll:
            if (gStopRequested) {
                beginPostRoll();
            } else if (millis() - gRecordFlowTimerMs >= kCameraPreRollMs) {
                gPendingCameraCommandId = queueCameraCommand(CameraCommandType::kTrigger);
                if (gPendingCameraCommandId == 0) {
                    gCameraStartOk = false;
                    gRecordFlowState = RecordFlowState::kActive;
                } else {
                    gRecordFlowState = RecordFlowState::kCameraStart;
                }
            }
            break;

        case RecordFlowState::kCameraStart:
            if (gCameraCompletedCommandId == gPendingCameraCommandId) {
                gCameraStartOk = gCameraLastCommandOk;
                gCameraRecordingAssumed = gCameraStartOk;
                gRecordFlowState = RecordFlowState::kActive;
                if (gStopRequested) {
                    beginCameraStop();
                }
            }
            break;

        case RecordFlowState::kActive:
            break;

        case RecordFlowState::kCameraStop:
            if (gCameraCompletedCommandId == gPendingCameraCommandId) {
                gCameraStopOk = gCameraLastCommandOk;
                gCameraRecordingAssumed = false;
                beginPostRoll();
            }
            break;

        case RecordFlowState::kPostRoll:
            if (millis() - gRecordFlowTimerMs >= kCameraPostRollMs) {
                requestStopRecording();
                gRecordFlowState = RecordFlowState::kClosing;
            }
            break;

        case RecordFlowState::kClosing:
            if (gSamplerState == SamplerState::kIdle && !gLogFile) {
                gRecordFlowState = RecordFlowState::kIdle;
                gStopRequested = false;
            }
            break;
    }
}

void updateSessionInput(const Keyboard_Class::KeysState& status) {
    for (const auto raw : status.word) {
        const char value = sanitizeSessionChar(static_cast<char>(raw));
        if (value != '\0' && gSessionInputLength < sizeof(gSessionInput) - 1) {
            gSessionInput[gSessionInputLength++] = value;
            gSessionInput[gSessionInputLength] = '\0';
        }
    }
    if (status.del && gSessionInputLength > 0) {
        --gSessionInputLength;
        gSessionInput[gSessionInputLength] = '\0';
        gSessionError = false;
    }
    if (status.enter) {
        createSessionFolder();
    }
}

void updateInputs() {
    if (xSemaphoreTake(gI2cMutex, pdMS_TO_TICKS(25)) == pdTRUE) {
        M5Cardputer.update();
        xSemaphoreGive(gI2cMutex);
    }

    const bool buttonPressed = M5Cardputer.BtnA.wasPressed();

    if (!gSessionConfirmed) {
        if (M5Cardputer.Keyboard.isChange() && M5Cardputer.Keyboard.isPressed()) {
            updateSessionInput(M5Cardputer.Keyboard.keysState());
        }
        if (buttonPressed) {
            createSessionFolder();
        }
        return;
    }

    bool recordPressed = false;
    bool calibrationPressed = false;
    bool diagnosticsPressed = false;
    bool sdRetryPressed = false;
    bool pairPressed = false;
    bool newSessionPressed = false;

    if (M5Cardputer.Keyboard.isChange() && M5Cardputer.Keyboard.isPressed()) {
        recordPressed = M5Cardputer.Keyboard.isKeyPressed('g') ||
                        M5Cardputer.Keyboard.isKeyPressed('G') ||
                        M5Cardputer.Keyboard.isKeyPressed(' ');
        calibrationPressed = M5Cardputer.Keyboard.isKeyPressed('c') ||
                             M5Cardputer.Keyboard.isKeyPressed('C');
        diagnosticsPressed = M5Cardputer.Keyboard.isKeyPressed('d') ||
                             M5Cardputer.Keyboard.isKeyPressed('D');
        sdRetryPressed = M5Cardputer.Keyboard.isKeyPressed('r') ||
                         M5Cardputer.Keyboard.isKeyPressed('R');
        pairPressed = M5Cardputer.Keyboard.isKeyPressed('p') ||
                      M5Cardputer.Keyboard.isKeyPressed('P');
        newSessionPressed = M5Cardputer.Keyboard.isKeyPressed('n') ||
                            M5Cardputer.Keyboard.isKeyPressed('N');
    }

    if (buttonPressed || recordPressed) {
        handleRecordControl();
    }
    if (calibrationPressed) {
        requestCalibration();
    }
    if (diagnosticsPressed && gRecordFlowState == RecordFlowState::kIdle) {
        gUiPage = gUiPage == UiPage::kMain ? UiPage::kDiagnostics : UiPage::kMain;
    }
    if (sdRetryPressed && gSamplerState == SamplerState::kIdle && !gLogFile) {
        gSdReady = initializeSd();
    }
    if (pairPressed && gSamplerState == SamplerState::kIdle && !gLogFile &&
        gRecordFlowState == RecordFlowState::kIdle &&
        gCameraState != CameraState::kPairing &&
        gCameraState != CameraState::kConnecting &&
        gCameraState != CameraState::kSending) {
        queueCameraCommand(CameraCommandType::kPair);
    }
    if (newSessionPressed && gSamplerState == SamplerState::kIdle && !gLogFile &&
        gRecordFlowState == RecordFlowState::kIdle) {
        openSessionEditor(true);
    }
}

bool isRecordingState(SamplerState state) {
    return state == SamplerState::kRecording || state == SamplerState::kStartRequest ||
           state == SamplerState::kStopRequest;
}

bool isTimeHealthy() {
    return gSensorTimeErrors == 0 && gSensorTimeFallbacks == 0;
}

const char* fileNameFromPath(const char* path) {
    if (path == nullptr) {
        return "--";
    }
    const char* name = std::strrchr(path, '/');
    return name == nullptr ? path : name + 1;
}

const char* cameraShortStatus() {
    switch (gCameraState) {
        case CameraState::kReady:
            return "READY";
        case CameraState::kUnpaired:
            return "PAIR";
        case CameraState::kConnecting:
            return "LINK";
        case CameraState::kPairing:
            return "PAIR";
        case CameraState::kSending:
            return "SEND";
        case CameraState::kFault:
            return "FAULT";
        default:
            return "OFF";
    }
}

const char* cameraLongStatus() {
    switch (gCameraState) {
        case CameraState::kReady:
            return "CAM READY";
        case CameraState::kUnpaired:
            return "P TO PAIR";
        case CameraState::kConnecting:
            return "CAM LINKING";
        case CameraState::kPairing:
            return "CAM PAIRING";
        case CameraState::kSending:
            return "CAM COMMAND";
        case CameraState::kFault:
            return "CAM RETRY";
        default:
            return "CAM OFFLINE";
    }
}

uint16_t cameraStatusColor() {
    switch (gCameraState) {
        case CameraState::kReady:
            return kUiGood;
        case CameraState::kConnecting:
        case CameraState::kPairing:
        case CameraState::kSending:
            return kUiCopper;
        case CameraState::kFault:
            return kUiFault;
        default:
            return kUiWarn;
    }
}

const char* flowStatusText() {
    switch (gRecordFlowState) {
        case RecordFlowState::kOpening:
            return "LOG START";
        case RecordFlowState::kPreRoll:
            return "PRE ROLL";
        case RecordFlowState::kCameraStart:
            return "CAM START";
        case RecordFlowState::kActive:
            return gCameraStartOk ? "REC ACTIVE" : "GYRO ONLY";
        case RecordFlowState::kCameraStop:
            return "CAM STOP";
        case RecordFlowState::kPostRoll:
            return "POST ROLL";
        case RecordFlowState::kClosing:
            return "FILE CLOSE";
        default:
            return "STANDBY";
    }
}

void formatCompactCounter(uint32_t value, char* output, size_t outputSize) {
    if (value >= 1000000) {
        std::snprintf(output, outputSize, "%luM",
                      static_cast<unsigned long>(value / 1000000));
    } else if (value >= 10000) {
        std::snprintf(output, outputSize, "%luK",
                      static_cast<unsigned long>(value / 1000));
    } else {
        std::snprintf(output, outputSize, "%lu", static_cast<unsigned long>(value));
    }
}

void sampleMotionForGraph() {
    uint32_t sequenceBefore = 0;
    uint32_t sequenceAfter = 0;
    int16_t values[3]{};
    do {
        sequenceBefore = gUiMotionSequence;
        values[0] = gUiGyroRaw[0];
        values[1] = gUiGyroRaw[1];
        values[2] = gUiGyroRaw[2];
        sequenceAfter = gUiMotionSequence;
    } while (sequenceBefore != sequenceAfter);

    if (sequenceAfter == gUiLastMotionSequence) {
        return;
    }
    for (size_t axis = 0; axis < 3; ++axis) {
        gUiGraph[axis][gUiGraphHead] = values[axis];
    }
    gUiGraphHead = (gUiGraphHead + 1) % kUiGraphSamples;
    gUiGraphCount = std::min(gUiGraphCount + 1, kUiGraphSamples);
    gUiLastMotionSequence = sequenceAfter;
}

void drawInstrumentPanel(int x, int y, int width, int height, uint16_t accent) {
    auto& display = uiDisplay();
    display.fillRect(x, y, width, height, kUiPanel);
    display.drawRect(x, y, width, height, kUiRule);
    display.drawFastHLine(x + 1, y + 1, 12, accent);
    display.drawFastVLine(x + 1, y + 1, 8, accent);
    display.drawFastHLine(x + width - 11, y + height - 2, 10, kUiRule);
}

void drawBatteryStatus() {
    auto& display = uiDisplay();
    const int level = static_cast<int>(
        std::max<int32_t>(0, std::min<int32_t>(100, gBatteryLevel)));
    const bool valid = gBatteryLevel >= 0;
    const bool charging = gChargeState == ChargeState::kCharging;
    uint16_t color = kUiGood;
    if (!valid) {
        color = kUiMuted;
    } else if (level <= 15) {
        color = kUiFault;
    } else if (level <= 30) {
        color = kUiWarn;
    } else if (charging) {
        color = kUiCopper;
    }

    display.setTextSize(1);
    display.setTextColor(charging ? kUiCopper : kUiMuted, kUiBackground);
    display.setCursor(148, 4);
    display.printf(charging ? "CHG" : "BAT");

    constexpr int batteryX = 177;
    constexpr int batteryY = 4;
    constexpr int batteryWidth = 25;
    constexpr int batteryHeight = 9;
    display.drawRect(batteryX, batteryY, batteryWidth, batteryHeight, color);
    display.fillRect(batteryX + batteryWidth, batteryY + 2, 2, batteryHeight - 4, color);
    if (valid) {
        const int innerWidth = batteryWidth - 4;
        const int fill = std::max(1, (level * innerWidth) / 100);
        display.fillRect(batteryX + 2, batteryY + 2, fill, batteryHeight - 4, color);
    }

    display.setTextColor(kUiText, kUiBackground);
    display.setCursor(208, 4);
    if (valid) {
        display.printf("%d%%", level);
    } else {
        display.printf("--%%");
    }
}

void drawHeader(const char* title) {
    auto& display = uiDisplay();
    display.fillRect(0, 0, display.width(), 19, kUiBackground);
    display.fillRect(6, 4, 3, 10, kUiCopper);
    display.setTextSize(1);
    display.setTextColor(kUiText, kUiBackground);
    display.setCursor(14, 3);
    display.printf("%s", title);
    display.setTextColor(kUiMuted, kUiBackground);
    display.setCursor(112, 4);
    display.printf("V%s", kFirmwareVersion);
    drawBatteryStatus();
    display.drawFastHLine(0, 18, display.width(), kUiRule);
    display.drawFastHLine(6, 18, 40, kUiCopper);
}

void drawMotionGraph(int x, int y, int width, int height) {
    auto& display = uiDisplay();
    sampleMotionForGraph();
    drawInstrumentPanel(x, y, width, height, kUiCopper);

    display.setTextSize(1);
    display.setTextColor(kUiMuted, kUiPanel);
    display.setCursor(x + 5, y + 3);
    display.printf("GYRO RATE");
    display.setTextColor(kUiAxisX, kUiPanel);
    display.setCursor(x + width - 36, y + 3);
    display.printf("X");
    display.setTextColor(kUiAxisY, kUiPanel);
    display.setCursor(x + width - 24, y + 3);
    display.printf("Y");
    display.setTextColor(kUiAxisZ, kUiPanel);
    display.setCursor(x + width - 12, y + 3);
    display.printf("Z");

    const int plotLeft = x + 4;
    const int plotRight = x + width - 4;
    const int plotTop = y + 14;
    const int plotBottom = y + height - 4;
    const int plotCenter = (plotTop + plotBottom) / 2;
    display.drawFastHLine(plotLeft, plotCenter, plotRight - plotLeft, kUiRule);
    for (int division = 1; division < 4; ++division) {
        const int gridX = plotLeft + ((plotRight - plotLeft) * division) / 4;
        for (int gridY = plotTop; gridY <= plotBottom; gridY += 4) {
            display.drawPixel(gridX, gridY, kUiRule);
        }
    }

    if (gUiGraphCount < 2) {
        return;
    }
    int32_t peak = 512;
    const size_t start = gUiGraphCount == kUiGraphSamples ? gUiGraphHead : 0;
    for (size_t axis = 0; axis < 3; ++axis) {
        for (size_t index = 0; index < gUiGraphCount; ++index) {
            const int32_t value = gUiGraph[axis][(start + index) % kUiGraphSamples];
            peak = std::max<int32_t>(peak, std::abs(value));
        }
    }
    const uint16_t colors[3] = {kUiAxisX, kUiAxisY, kUiAxisZ};
    const int amplitude = std::max(1, (plotBottom - plotTop) / 2 - 1);
    for (size_t axis = 0; axis < 3; ++axis) {
        bool havePrevious = false;
        int previousX = 0;
        int previousY = 0;
        for (size_t index = 0; index < gUiGraphCount; ++index) {
            const size_t historyIndex = (start + index) % kUiGraphSamples;
            const int32_t value = gUiGraph[axis][historyIndex];
            const int pointX = plotRight -
                static_cast<int>(((gUiGraphCount - 1 - index) *
                                  static_cast<size_t>(plotRight - plotLeft)) /
                                 (kUiGraphSamples - 1));
            const int pointY = plotCenter - static_cast<int>((value * amplitude) / peak);
            if (havePrevious) {
                display.drawLine(previousX, previousY, pointX, pointY, colors[axis]);
            }
            previousX = pointX;
            previousY = pointY;
            havePrevious = true;
        }
    }
}

void drawMetric(int x, int y, const char* label, const char* value, uint16_t valueColor) {
    auto& display = uiDisplay();
    constexpr int width = 55;
    constexpr int height = 27;
    display.fillRect(x, y, width, height, kUiPanelRaised);
    display.drawFastHLine(x, y, width, kUiRule);
    display.fillRect(x, y, 9, 2, valueColor);
    display.setTextSize(1);
    display.setTextColor(kUiMuted, kUiPanelRaised);
    display.setCursor(x + 4, y + 4);
    display.printf("%s", label);
    display.setTextColor(valueColor, kUiPanelRaised);
    display.setCursor(x + 4, y + 14);
    display.printf("%s", value);
}

void drawMetricRow(bool recording) {
    char value0[16];
    char value2[16];
    char value3[16];
    if (recording) {
        std::snprintf(value0, sizeof(value0), "%u.%u", gRateTimes10 / 10,
                      gRateTimes10 % 10);
        formatCompactCounter(gMissedSensorSamples, value2, sizeof(value2));
        formatCompactCounter(gDroppedQueueSamples, value3, sizeof(value3));
        drawMetric(7, 78, "RATE HZ", value0,
                   (gRateTimes10 >= 7950 && gRateTimes10 <= 8050) ?
                       kUiGood : kUiWarn);
        drawMetric(64, 78, "CLOCK", isTimeHealthy() ? "LOCK" : "WARN",
                   isTimeHealthy() ? kUiGood : kUiFault);
        drawMetric(121, 78, "MISS", value2,
                   gMissedSensorSamples == 0 ? kUiText : kUiFault);
        drawMetric(178, 78, "DROP", value3,
                   gDroppedQueueSamples == 0 ? kUiText : kUiFault);
    } else {
        std::snprintf(value0, sizeof(value0), "%lu",
                      static_cast<unsigned long>(kSampleRateHz));
        std::snprintf(value2, sizeof(value2), "%s", gSdReady ? "READY" : "FAULT");
        drawMetric(7, 78, "ODR HZ", value0, kUiText);
        drawMetric(64, 78, "AXIS", kOrientation, kUiText);
        drawMetric(121, 78, "MEDIA", value2, gSdReady ? kUiGood : kUiFault);
        drawMetric(178, 78, "CAM", cameraShortStatus(), cameraStatusColor());
    }
}

void drawStatePanel(bool recording) {
    auto& display = uiDisplay();
    const uint16_t accent = recording ? kUiFault : (gSdReady ? kUiGood : kUiFault);
    drawInstrumentPanel(7, 23, 72, 51, accent);
    display.setTextSize(1);
    display.setTextColor(kUiMuted, kUiPanel);
    display.setCursor(13, 28);
    display.printf(recording ? "SEQUENCE" : "SYSTEM");

    if (recording) {
        const uint32_t totalSeconds = gRecordingElapsedMs / 1000;
        display.setTextSize(2);
        display.setTextColor(kUiText, kUiPanel);
        display.setCursor(12, 41);
        display.printf("%02lu:%02lu", static_cast<unsigned long>(totalSeconds / 60),
                       static_cast<unsigned long>(totalSeconds % 60));
        display.fillRect(13, 64, 5, 5, kUiFault);
        display.setTextSize(1);
        display.setTextColor(kUiFault, kUiPanel);
        display.setCursor(19, 62);
        display.printf("%.10s", flowStatusText());
    } else {
        display.setTextSize(gSdReady ? 2 : 1);
        display.setTextColor(accent, kUiPanel);
        display.setCursor(12, 42);
        display.printf(gSdReady ? "ARMED" : "NO MEDIA");
        display.setTextSize(1);
        display.setTextColor(cameraStatusColor(), kUiPanel);
        display.setCursor(12, 63);
        display.printf("%.11s", cameraLongStatus());
    }
}

void drawInfoStrip(bool recording) {
    auto& display = uiDisplay();
    display.fillRect(7, 109, 226, 14, kUiPanel);
    display.drawFastVLine(7, 109, 14, recording ? kUiFault : kUiCopper);
    display.setTextSize(1);
    display.setTextColor(kUiText, kUiPanel);
    display.setCursor(12, 112);
    if (recording) {
        char samples[16];
        formatCompactCounter(gCapturedSamples, samples, sizeof(samples));
        display.printf("FILE %.18s", fileNameFromPath(gLogPath));
        display.setTextColor(kUiMuted, kUiPanel);
        display.setCursor(181, 112);
        display.printf("%s", samples);
    } else if (gLastLogPath[0] != '\0') {
        display.printf("LAST %.18s", fileNameFromPath(gLastLogPath));
        display.setTextColor(gLastWriteFailed ? kUiFault : kUiMuted, kUiPanel);
        display.setCursor(190, 112);
        display.printf("%luK",
                       static_cast<unsigned long>((gLastFileBytes + 1023) / 1024));
    } else {
        display.printf("SESSION %.20s", gSessionName);
    }
}

void drawFooter(bool recording, const char* overrideText = nullptr) {
    auto& display = uiDisplay();
    display.setTextSize(1);
    display.setTextColor(kUiMuted, kUiBackground);
    display.setCursor(7, 127);
    if (overrideText != nullptr) {
        display.printf("%s", overrideText);
    } else if (recording) {
        display.printf("GO STOP   1S PRE/POST   D LOCKED");
    } else {
        display.printf("GO REC   P PAIR   N SESSION   D SYS");
    }
}

void drawSegmentedProgress(uint32_t value, uint32_t maximum) {
    auto& display = uiDisplay();
    constexpr int segments = 12;
    constexpr int x = 14;
    constexpr int y = 79;
    constexpr int segmentWidth = 16;
    constexpr int gap = 2;
    const int active = maximum == 0 ? 0 :
        static_cast<int>((static_cast<uint64_t>(value) * segments) / maximum);
    for (int segment = 0; segment < segments; ++segment) {
        const int segmentX = x + segment * (segmentWidth + gap);
        display.fillRect(segmentX, y, segmentWidth, 9,
                         segment < active ? kUiCopper : kUiPanelRaised);
        display.drawRect(segmentX, y, segmentWidth, 9, kUiRule);
    }
}

void drawSessionPage() {
    auto& display = uiDisplay();
    drawHeader("SESSION SETUP");
    drawInstrumentPanel(7, 24, 226, 91, gSessionError ? kUiFault : kUiCopper);
    display.setTextSize(1);
    display.setTextColor(kUiMuted, kUiPanel);
    display.setCursor(14, 31);
    display.printf("NEW SHOOT FOLDER");

    display.setTextSize(2);
    display.setTextColor(gSessionError ? kUiFault : kUiText, kUiPanel);
    display.setCursor(14, 50);
    if (gSessionInputLength > 0) {
        display.printf("%.12s", gSessionInput);
    } else {
        display.printf("_");
    }

    display.drawFastHLine(14, 72, 210, gSessionError ? kUiFault : kUiCopper);
    display.setTextSize(1);
    display.setTextColor(kUiMuted, kUiPanel);
    display.setCursor(14, 82);
    if (gSessionError) {
        display.printf(gSdReady ? "NAME EXISTS LIMIT OR SD ERROR" : "INSERT MICROSD CARD");
    } else {
        display.printf("FILES: NAME_001.GCSV  NAME_002.GCSV");
    }
    display.setTextColor(kUiCopper, kUiPanel);
    display.setCursor(14, 99);
    display.printf("ENTER OR GO TO CREATE");
    drawFooter(false, "TYPE NAME   DEL ERASE   ENTER OK");
}

void drawCalibrationPage() {
    auto& display = uiDisplay();
    drawHeader("GYRO CALIBRATION");
    drawInstrumentPanel(7, 24, 226, 91, kUiCopper);
    display.setTextSize(1);
    display.setTextColor(kUiMuted, kUiPanel);
    display.setCursor(14, 30);
    display.printf("BMI270 ZERO CALIBRATION");
    display.setTextSize(2);
    display.setTextColor(kUiText, kUiPanel);
    display.setCursor(14, 45);
    display.printf("HOLD STILL");
    display.setTextSize(1);
    display.setTextColor(kUiCopper, kUiPanel);
    display.setCursor(14, 64);
    display.printf("%lu OF %lu SAMPLES", static_cast<unsigned long>(gCalibrationCount),
                   static_cast<unsigned long>(kCalibrationSamples));

    display.drawCircle(204, 51, 13, kUiRule);
    display.drawCircle(204, 51, 8, kUiRule);
    display.drawFastHLine(187, 51, 34, kUiCopper);
    display.drawFastVLine(204, 34, 35, kUiCopper);
    display.fillRect(202, 49, 5, 5, kUiText);

    drawSegmentedProgress(gCalibrationCount, kCalibrationSamples);
    display.setTextColor(gCalibrationRetries == 0 ? kUiMuted : kUiWarn, kUiPanel);
    display.setCursor(14, 98);
    if (gCalibrationRetries == 0) {
        display.printf("MOTION CHECK   NOISE %u", gCalibrationStdRaw);
    } else {
        display.printf("MOTION DETECTED   RETRY %lu",
                       static_cast<unsigned long>(gCalibrationRetries));
    }
    drawFooter(false, "KEEP STILL   AUTO FINISH");
}

void drawErrorPage() {
    auto& display = uiDisplay();
    drawHeader("GYRO FAULT");
    drawInstrumentPanel(7, 24, 226, 91, kUiFault);
    display.setTextSize(1);
    display.setTextColor(kUiFault, kUiPanel);
    display.setCursor(14, 31);
    display.printf("SENSOR ERROR");
    display.setTextSize(2);
    display.setTextColor(kUiText, kUiPanel);
    display.setCursor(14, 47);
    display.printf("BMI270 E%02d", static_cast<int>(gSensorError));
    display.setTextSize(1);
    display.setTextColor(kUiMuted, kUiPanel);
    display.setCursor(14, 75);
    display.printf("CONFIGURATION  %s", gConfigVerified ? "VERIFIED" : "FAILED");
    display.setCursor(14, 93);
    display.printf("POWER CYCLE DEVICE");
    drawFooter(false, "RECORDING LOCKED   SENSOR FAULT");
}

void drawMainPage() {
    if (!gSessionConfirmed) {
        drawSessionPage();
        return;
    }

    const SamplerState state = gSamplerState;
    if (state == SamplerState::kCalibrating ||
        state == SamplerState::kCalibrationRequest) {
        drawCalibrationPage();
        return;
    }
    if (state == SamplerState::kError) {
        drawErrorPage();
        return;
    }

    const bool recording = isRecordingState(state) ||
                           gRecordFlowState != RecordFlowState::kIdle;
    drawHeader(recording ? "GYRO + CAMERA" : "GYRO STANDBY");
    drawStatePanel(recording);
    drawMotionGraph(86, 23, 147, 51);
    drawMetricRow(recording);
    drawInfoStrip(recording);
    drawFooter(recording);
}

void drawDiagnosticRow(int y, const char* label, const char* value, uint16_t color) {
    auto& display = uiDisplay();
    display.drawFastHLine(12, y + 9, 216, kUiRule);
    display.setTextSize(1);
    display.setTextColor(kUiMuted, kUiPanel);
    display.setCursor(14, y);
    display.printf("%s", label);
    display.setTextColor(color, kUiPanel);
    display.setCursor(62, y);
    display.printf("%s", value);
}

void drawDiagnosticsPage() {
    auto& display = uiDisplay();
    drawHeader("GYRO SYSTEM");
    drawInstrumentPanel(7, 23, 226, 96, kUiCopper);
    char value[96];

    drawDiagnosticRow(25, "SENSOR", "BMI270  0x69  I2C 1MHz", kUiText);
    drawDiagnosticRow(36, "PROFILE", "800Hz  2000dps  OSR4 HP", kUiText);
    std::snprintf(value, sizeof(value), "%u.%uHz F%lu B%lu E%lu",
                  gRateTimes10 / 10, gRateTimes10 % 10,
                  static_cast<unsigned long>(gSensorTimeFrames),
                  static_cast<unsigned long>(gSensorTimeFallbacks),
                  static_cast<unsigned long>(gSensorTimeErrors));
    drawDiagnosticRow(47, "CLOCK", value, isTimeHealthy() ? kUiGood : kUiFault);
    std::snprintf(value, sizeof(value), "MISS %lu SKIP %lu DROP %lu C%lu",
                  static_cast<unsigned long>(gMissedSensorSamples),
                  static_cast<unsigned long>(gSkippedFifoSamples),
                  static_cast<unsigned long>(gDroppedQueueSamples),
                  static_cast<unsigned long>(gClockCorrections));
    drawDiagnosticRow(58, "LOSS", value,
                      (gMissedSensorSamples == 0 && gDroppedQueueSamples == 0) ?
                          kUiGood : kUiFault);
    std::snprintf(value, sizeof(value), "%u/%uB HIGH %u WARN %lu",
                  gFifoCurrentBytes, kFifoHardwareBytes, gFifoHighWaterBytes,
                  static_cast<unsigned long>(gFifoWarnings));
    drawDiagnosticRow(69, "FIFO", value, gFifoWarnings == 0 ? kUiText : kUiWarn);
    std::snprintf(value, sizeof(value), "%ld %ld %ld / N%u",
                  static_cast<long>(gGyroBias[0]), static_cast<long>(gGyroBias[1]),
                  static_cast<long>(gGyroBias[2]), gCalibrationStdRaw);
    drawDiagnosticRow(80, "BIAS", value, kUiText);
    std::snprintf(value, sizeof(value), "%s  PRE/POST 1.0S", cameraLongStatus());
    drawDiagnosticRow(91, "CAMERA", value, cameraStatusColor());
    if (gBatteryMillivolts > 0 && gBatteryLevel >= 0) {
        std::snprintf(value, sizeof(value), "%ld%% %d.%03dV %s%s",
                      static_cast<long>(gBatteryLevel), gBatteryMillivolts / 1000,
                      gBatteryMillivolts % 1000,
                      gChargeState == ChargeState::kCharging ? "CHARGING" : "BATTERY",
                      gChargeStateEstimated ? " EST" : "");
    } else {
        std::snprintf(value, sizeof(value), "TELEMETRY UNAVAILABLE");
    }
    drawDiagnosticRow(102, "POWER", value,
                      gChargeState == ChargeState::kCharging ? kUiCopper : kUiText);
    drawFooter(false, "D BACK   C CAL   GREEN IS NORMAL");
}

void drawStatus() {
    auto& display = uiDisplay();
    display.startWrite();
    display.fillScreen(kUiBackground);
    display.setTextWrap(false);
    display.setFont(&fonts::Font0);
    display.setTextSize(1);
    if (gUiPage == UiPage::kDiagnostics && gSessionConfirmed) {
        drawDiagnosticsPage();
    } else {
        drawMainPage();
    }
    display.endWrite();
    if (gUiCanvasReady) {
        gUiCanvas.pushSprite(0, 0);
    }
}

}  // namespace

void setup() {
    auto config = M5.config();
    config.fallback_board = m5::board_t::board_M5CardputerADV;
    config.internal_imu = false;
    config.internal_rtc = false;
    config.internal_mic = false;
    config.internal_spk = false;
    config.clear_display = true;
    config.serial_baudrate = 115200;
    M5Cardputer.begin(config, true);

    M5Cardputer.Display.setRotation(1);
    M5Cardputer.Display.setBrightness(150);
    M5Cardputer.Display.setTextWrap(false);
    updatePowerTelemetry(true);

    gI2cMutex = xSemaphoreCreateMutex();
    gSampleQueue = xQueueCreate(kSampleQueueLength, sizeof(LogSample));
    gCameraQueue = xQueueCreate(4, sizeof(CameraCommand));
    if (gI2cMutex == nullptr || gSampleQueue == nullptr || gCameraQueue == nullptr) {
        gSensorError = BMI2_E_NULL_PTR;
        gSamplerState = SamplerState::kError;
        drawStatus();
        return;
    }

    gUiCanvas.setColorDepth(8);
    gUiCanvasReady = gUiCanvas.createSprite(
        M5Cardputer.Display.width(), M5Cardputer.Display.height()) != nullptr;
    if (gUiCanvasReady) {
        gUiCanvas.setTextWrap(false);
        gUiCanvas.setFont(&fonts::Font0);
        gUiCanvas.setTextSize(1);
        gUiDisplay = &gUiCanvas;
    }

    gSdReady = initializeSd();
    if (!configureBmi270()) {
        drawStatus();
        return;
    }

    const BaseType_t samplerCreated = xTaskCreatePinnedToCore(
        samplerTask, "bmi270-timed-fifo", 8192, nullptr, 5, &gSamplerTask, 0);
    if (samplerCreated != pdPASS) {
        gSensorError = BMI2_E_NULL_PTR;
        gSamplerState = SamplerState::kError;
        drawStatus();
        return;
    }

    const bool bleInitialized = gCanonRemote.begin();
    if (!bleInitialized) {
        gCameraState = CameraState::kFault;
    } else {
        gCameraState = gCanonRemote.hasPairedCamera() ?
            CameraState::kDisconnected : CameraState::kUnpaired;
        const BaseType_t cameraCreated = xTaskCreatePinnedToCore(
            cameraTask, "canon-ble", 8192, nullptr, 1, &gCameraTask, 1);
        if (cameraCreated != pdPASS) {
            gCameraState = CameraState::kFault;
        } else if (gCanonRemote.hasPairedCamera()) {
            queueCameraCommand(CameraCommandType::kConnect);
        }
    }

    drawStatus();
}

void loop() {
    updateInputs();
    serviceRecordFlow();
    serviceLogWriter();
    finishRecordingIfReady();
    serviceRecordFlow();
    updatePowerTelemetry();

    if (gWriteFailed && gLogFile) {
        requestStopRecording();
        gRecordFlowState = RecordFlowState::kClosing;
    }

    static uint32_t lastDrawMs = 0;
    const uint32_t now = millis();
    if (now - lastDrawMs >= 200) {
        drawStatus();
        lastDrawMs = now;
    }
    delay(2);
}
