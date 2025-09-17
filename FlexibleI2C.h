#ifndef FLEXIBLE_I2C_H
#define FLEXIBLE_I2C_H

#include <Arduino.h>
#include <Wire.h>
#include <FlexibleEndpoints.h>
#include <ArduinoJson.h>
#include <vector>
#include <map>

struct I2CBusConfig {
    uint8_t sda_pin;
    uint8_t scl_pin;
    uint32_t frequency;
    TwoWire* wire_instance;
    bool initialized;

    I2CBusConfig() : sda_pin(255), scl_pin(255), frequency(100000), wire_instance(nullptr), initialized(false) {}
    I2CBusConfig(uint8_t sda, uint8_t scl, uint32_t freq = 100000)
        : sda_pin(sda), scl_pin(scl), frequency(freq), wire_instance(nullptr), initialized(false) {}
};

struct I2CDeviceInfo {
    uint8_t address;
    uint8_t bus_id;
    String device_name;
    bool responsive;
    unsigned long last_seen;

    I2CDeviceInfo(uint8_t addr, uint8_t bus, String name = "")
        : address(addr), bus_id(bus), device_name(name), responsive(false), last_seen(0) {}
};

class FlexibleI2C {
public:
    FlexibleI2C();
    virtual ~FlexibleI2C();

    // Initialize with FlexibleEndpoints integration
    void init(FlexibleEndpoints& endpoints);

    // Bus management
    bool initBus(uint8_t bus_id, uint8_t sda_pin, uint8_t scl_pin, uint32_t frequency = 100000);
    bool isBusInitialized(uint8_t bus_id);
    TwoWire* getBus(uint8_t bus_id);

    // Device scanning and management
    std::vector<uint8_t> scanBus(uint8_t bus_id);
    std::vector<I2CDeviceInfo> getAllDevices();
    bool isDevicePresent(uint8_t bus_id, uint8_t address);

    // Basic I2C operations
    bool writeRegister(uint8_t bus_id, uint8_t device_address, uint8_t reg_address, uint8_t data);
    bool writeRegister16(uint8_t bus_id, uint8_t device_address, uint8_t reg_address, uint16_t data);
    bool writeBytes(uint8_t bus_id, uint8_t device_address, uint8_t reg_address, const uint8_t* data, size_t length);

    uint8_t readRegister(uint8_t bus_id, uint8_t device_address, uint8_t reg_address);
    uint16_t readRegister16(uint8_t bus_id, uint8_t device_address, uint8_t reg_address);
    bool readBytes(uint8_t bus_id, uint8_t device_address, uint8_t reg_address, uint8_t* data, size_t length);

    // Raw I2C operations
    bool beginTransmission(uint8_t bus_id, uint8_t address);
    bool endTransmission(uint8_t bus_id, bool stop = true);
    bool requestFrom(uint8_t bus_id, uint8_t address, uint8_t quantity, bool stop = true);

    // Virtual methods for extensibility
    virtual void onDeviceFound(uint8_t bus_id, uint8_t address) {}
    virtual void onDeviceLost(uint8_t bus_id, uint8_t address) {}
    virtual void registerCustomEndpoints(FlexibleEndpoints& endpoints) {}

    // Configuration
    void setTimeout(uint16_t timeout_ms) { i2c_timeout = timeout_ms; }
    uint16_t getTimeout() const { return i2c_timeout; }

    // Error handling
    enum I2CError {
        SUCCESS = 0,
        TIMEOUT = 1,
        NACK_ADDRESS = 2,
        NACK_DATA = 3,
        OTHER_ERROR = 4,
        BUS_NOT_INITIALIZED = 5,
        INVALID_PARAMETERS = 6
    };

    I2CError getLastError() const { return last_error; }
    String getErrorString(I2CError error);

protected:
    std::map<uint8_t, I2CBusConfig> buses;
    std::vector<I2CDeviceInfo> known_devices;
    uint16_t i2c_timeout;
    I2CError last_error;
    FlexibleEndpoints* endpoints_ptr;

    void setError(I2CError error) { last_error = error; }
    bool validateBusAndAddress(uint8_t bus_id, uint8_t address);
    void registerBuiltinEndpoints(FlexibleEndpoints& endpoints);

    // Endpoint handlers
    std::pair<String, int> handleScanBus(std::map<String, String>& params);
    std::pair<String, int> handleInitBus(std::map<String, String>& params);
    std::pair<String, int> handleDeviceInfo(std::map<String, String>& params);
    std::pair<String, int> handleReadRegister(std::map<String, String>& params);
    std::pair<String, int> handleWriteRegister(std::map<String, String>& params);
    std::pair<String, int> handlePingDevice(std::map<String, String>& params);
    std::pair<String, int> handleReadBytes(std::map<String, String>& params);
    std::pair<String, int> handleWriteBytes(std::map<String, String>& params);

    // Helper methods
    DynamicJsonDocument deviceInfoToJson(const I2CDeviceInfo& device);
    DynamicJsonDocument busConfigToJson(uint8_t bus_id);
};

#endif // FLEXIBLE_I2C_H