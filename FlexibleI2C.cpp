#include "FlexibleI2C.h"

FlexibleI2C::FlexibleI2C() : i2c_timeout(1000), last_error(SUCCESS), endpoints_ptr(nullptr) {
}

FlexibleI2C::~FlexibleI2C() {
    for (auto& bus_pair : buses) {
        I2CBusConfig& config = bus_pair.second;
        if (config.initialized && config.wire_instance) {
            config.wire_instance->end();
        }
    }
}

void FlexibleI2C::init(FlexibleEndpoints& endpoints) {
    endpoints_ptr = &endpoints;
    endpoints.setLibraryName("FlexibleI2C");
    registerBuiltinEndpoints(endpoints);
    registerCustomEndpoints(endpoints);
}

bool FlexibleI2C::initBus(uint8_t bus_id, uint8_t sda_pin, uint8_t scl_pin, uint32_t frequency) {
    if (bus_id > 1) {
        setError(INVALID_PARAMETERS);
        return false;
    }

    if (buses.find(bus_id) != buses.end() && buses[bus_id].initialized) {
        return true;
    }

    TwoWire* wire_instance = (bus_id == 0) ? &Wire : &Wire1;

    I2CBusConfig config(sda_pin, scl_pin, frequency);
    config.wire_instance = wire_instance;

    bool success = wire_instance->begin(sda_pin, scl_pin, frequency);
    if (success) {
        config.initialized = true;
        buses[bus_id] = config;
        setError(SUCCESS);
        return true;
    } else {
        setError(OTHER_ERROR);
        return false;
    }
}

bool FlexibleI2C::isBusInitialized(uint8_t bus_id) {
    auto it = buses.find(bus_id);
    return (it != buses.end() && it->second.initialized);
}

TwoWire* FlexibleI2C::getBus(uint8_t bus_id) {
    auto it = buses.find(bus_id);
    if (it != buses.end() && it->second.initialized) {
        return it->second.wire_instance;
    }
    return nullptr;
}

std::vector<uint8_t> FlexibleI2C::scanBus(uint8_t bus_id) {
    std::vector<uint8_t> found_addresses;

    if (!isBusInitialized(bus_id)) {
        setError(BUS_NOT_INITIALIZED);
        return found_addresses;
    }

    TwoWire* wire = getBus(bus_id);
    if (!wire) {
        setError(BUS_NOT_INITIALIZED);
        return found_addresses;
    }

    for (uint8_t address = 1; address < 127; address++) {
        wire->beginTransmission(address);
        uint8_t error = wire->endTransmission();

        if (error == 0) {
            found_addresses.push_back(address);
            onDeviceFound(bus_id, address);

            bool device_exists = false;
            for (auto& device : known_devices) {
                if (device.address == address && device.bus_id == bus_id) {
                    device.responsive = true;
                    device.last_seen = millis();
                    device_exists = true;
                    break;
                }
            }

            if (!device_exists) {
                I2CDeviceInfo new_device(address, bus_id, "Unknown Device");
                new_device.responsive = true;
                new_device.last_seen = millis();
                known_devices.push_back(new_device);
            }
        }
    }

    for (auto& device : known_devices) {
        if (device.bus_id == bus_id) {
            bool found = false;
            for (uint8_t addr : found_addresses) {
                if (addr == device.address) {
                    found = true;
                    break;
                }
            }
            if (!found && device.responsive) {
                device.responsive = false;
                onDeviceLost(bus_id, device.address);
            }
        }
    }

    setError(SUCCESS);
    return found_addresses;
}

std::vector<I2CDeviceInfo> FlexibleI2C::getAllDevices() {
    return known_devices;
}

bool FlexibleI2C::isDevicePresent(uint8_t bus_id, uint8_t address) {
    if (!validateBusAndAddress(bus_id, address)) {
        return false;
    }

    TwoWire* wire = getBus(bus_id);
    wire->beginTransmission(address);
    uint8_t error = wire->endTransmission();

    return (error == 0);
}

bool FlexibleI2C::writeRegister(uint8_t bus_id, uint8_t device_address, uint8_t reg_address, uint8_t data) {
    if (!validateBusAndAddress(bus_id, device_address)) {
        return false;
    }

    TwoWire* wire = getBus(bus_id);
    wire->beginTransmission(device_address);
    wire->write(reg_address);
    wire->write(data);
    uint8_t error = wire->endTransmission();

    if (error == 0) {
        setError(SUCCESS);
        return true;
    } else {
        setError(static_cast<I2CError>(error));
        return false;
    }
}

bool FlexibleI2C::writeRegister16(uint8_t bus_id, uint8_t device_address, uint8_t reg_address, uint16_t data) {
    if (!validateBusAndAddress(bus_id, device_address)) {
        return false;
    }

    TwoWire* wire = getBus(bus_id);
    wire->beginTransmission(device_address);
    wire->write(reg_address);
    wire->write(data >> 8);
    wire->write(data & 0xFF);
    uint8_t error = wire->endTransmission();

    if (error == 0) {
        setError(SUCCESS);
        return true;
    } else {
        setError(static_cast<I2CError>(error));
        return false;
    }
}

bool FlexibleI2C::writeBytes(uint8_t bus_id, uint8_t device_address, uint8_t reg_address, const uint8_t* data, size_t length) {
    if (!validateBusAndAddress(bus_id, device_address) || !data || length == 0) {
        setError(INVALID_PARAMETERS);
        return false;
    }

    TwoWire* wire = getBus(bus_id);
    wire->beginTransmission(device_address);
    wire->write(reg_address);
    for (size_t i = 0; i < length; i++) {
        wire->write(data[i]);
    }
    uint8_t error = wire->endTransmission();

    if (error == 0) {
        setError(SUCCESS);
        return true;
    } else {
        setError(static_cast<I2CError>(error));
        return false;
    }
}

uint8_t FlexibleI2C::readRegister(uint8_t bus_id, uint8_t device_address, uint8_t reg_address) {
    if (!validateBusAndAddress(bus_id, device_address)) {
        return 0;
    }

    TwoWire* wire = getBus(bus_id);

    wire->beginTransmission(device_address);
    wire->write(reg_address);
    uint8_t error = wire->endTransmission(false);

    if (error != 0) {
        setError(static_cast<I2CError>(error));
        return 0;
    }

    uint8_t bytes_received = wire->requestFrom(device_address, (uint8_t)1, (uint8_t)true);
    if (bytes_received == 1) {
        setError(SUCCESS);
        return wire->read();
    } else {
        setError(TIMEOUT);
        return 0;
    }
}

uint16_t FlexibleI2C::readRegister16(uint8_t bus_id, uint8_t device_address, uint8_t reg_address) {
    if (!validateBusAndAddress(bus_id, device_address)) {
        return 0;
    }

    TwoWire* wire = getBus(bus_id);

    wire->beginTransmission(device_address);
    wire->write(reg_address);
    uint8_t error = wire->endTransmission(false);

    if (error != 0) {
        setError(static_cast<I2CError>(error));
        return 0;
    }

    uint8_t bytes_received = wire->requestFrom(device_address, (uint8_t)2, (uint8_t)true);
    if (bytes_received == 2) {
        uint16_t result = wire->read() << 8;
        result |= wire->read();
        setError(SUCCESS);
        return result;
    } else {
        setError(TIMEOUT);
        return 0;
    }
}

bool FlexibleI2C::readBytes(uint8_t bus_id, uint8_t device_address, uint8_t reg_address, uint8_t* data, size_t length) {
    if (!validateBusAndAddress(bus_id, device_address) || !data || length == 0) {
        setError(INVALID_PARAMETERS);
        return false;
    }

    TwoWire* wire = getBus(bus_id);

    wire->beginTransmission(device_address);
    wire->write(reg_address);
    uint8_t error = wire->endTransmission(false);

    if (error != 0) {
        setError(static_cast<I2CError>(error));
        return false;
    }

    uint8_t bytes_received = wire->requestFrom(device_address, (uint8_t)length, (uint8_t)true);
    if (bytes_received == length) {
        for (size_t i = 0; i < length; i++) {
            data[i] = wire->read();
        }
        setError(SUCCESS);
        return true;
    } else {
        setError(TIMEOUT);
        return false;
    }
}

bool FlexibleI2C::beginTransmission(uint8_t bus_id, uint8_t address) {
    if (!validateBusAndAddress(bus_id, address)) {
        return false;
    }

    TwoWire* wire = getBus(bus_id);
    wire->beginTransmission(address);
    return true;
}

bool FlexibleI2C::endTransmission(uint8_t bus_id, bool stop) {
    if (!isBusInitialized(bus_id)) {
        setError(BUS_NOT_INITIALIZED);
        return false;
    }

    TwoWire* wire = getBus(bus_id);
    uint8_t error = wire->endTransmission(stop);

    if (error == 0) {
        setError(SUCCESS);
        return true;
    } else {
        setError(static_cast<I2CError>(error));
        return false;
    }
}

bool FlexibleI2C::requestFrom(uint8_t bus_id, uint8_t address, uint8_t quantity, bool stop) {
    if (!validateBusAndAddress(bus_id, address)) {
        return false;
    }

    TwoWire* wire = getBus(bus_id);
    uint8_t bytes_received = wire->requestFrom(address, quantity, (uint8_t)stop);

    return (bytes_received == quantity);
}

String FlexibleI2C::getErrorString(I2CError error) {
    switch (error) {
        case SUCCESS: return "Success";
        case TIMEOUT: return "Timeout";
        case NACK_ADDRESS: return "NACK on address";
        case NACK_DATA: return "NACK on data";
        case OTHER_ERROR: return "Other error";
        case BUS_NOT_INITIALIZED: return "Bus not initialized";
        case INVALID_PARAMETERS: return "Invalid parameters";
        default: return "Unknown error";
    }
}

bool FlexibleI2C::validateBusAndAddress(uint8_t bus_id, uint8_t address) {
    if (!isBusInitialized(bus_id)) {
        setError(BUS_NOT_INITIALIZED);
        return false;
    }

    if (address == 0 || address > 127) {
        setError(INVALID_PARAMETERS);
        return false;
    }

    return true;
}

void FlexibleI2C::registerBuiltinEndpoints(FlexibleEndpoints& endpoints) {
    endpoints.addEndpoint(FLEXIBLE_ENDPOINT()
        .route("/initI2C")
        .summary("Initialize I2C bus")
        .description("Initialize an I2C bus with specified pins and frequency")
        .params({
            REQUIRED_INT_PARAM("bus_id", "Bus ID (0 or 1)"),
            REQUIRED_INT_PARAM("sda_pin", "SDA pin number"),
            REQUIRED_INT_PARAM("scl_pin", "SCL pin number"),
            INT_PARAM("frequency", "Bus frequency in Hz (default 100000)")
        })
        .responseType(JSON_RESPONSE)
        .handler([this](std::map<String, String>& params) {
            return handleInitBus(params);
        })
    );

    endpoints.addEndpoint(FLEXIBLE_ENDPOINT()
        .route("/scanI2C")
        .summary("Scan I2C bus for devices")
        .description("Scan the specified I2C bus for responsive devices")
        .params({
            REQUIRED_INT_PARAM("bus_id", "Bus ID to scan")
        })
        .responseType(JSON_RESPONSE)
        .handler([this](std::map<String, String>& params) {
            return handleScanBus(params);
        })
    );

    endpoints.addEndpoint(FLEXIBLE_ENDPOINT()
        .route("/getI2CDevices")
        .summary("Get all known devices")
        .description("Get information about all known I2C devices")
        .responseType(JSON_RESPONSE)
        .handler([this](std::map<String, String>& params) {
            return handleDeviceInfo(params);
        })
    );

    endpoints.addEndpoint(FLEXIBLE_ENDPOINT()
        .route("/readI2C")
        .summary("Read register from device")
        .description("Read a register value from an I2C device")
        .params({
            REQUIRED_INT_PARAM("bus_id", "Bus ID"),
            REQUIRED_STR_PARAM("device_addr", "Device address (hex format, e.g., '0x48')"),
            REQUIRED_STR_PARAM("reg_addr", "Register address (hex format)")
        })
        .responseType(JSON_RESPONSE)
        .handler([this](std::map<String, String>& params) {
            return handleReadRegister(params);
        })
    );

    endpoints.addEndpoint(FLEXIBLE_ENDPOINT()
        .route("/writeI2C")
        .summary("Write register to device")
        .description("Write a value to a register on an I2C device")
        .params({
            REQUIRED_INT_PARAM("bus_id", "Bus ID"),
            REQUIRED_STR_PARAM("device_addr", "Device address (hex format)"),
            REQUIRED_STR_PARAM("reg_addr", "Register address (hex format)"),
            REQUIRED_STR_PARAM("value", "Value to write (hex format)")
        })
        .responseType(JSON_RESPONSE)
        .handler([this](std::map<String, String>& params) {
            return handleWriteRegister(params);
        })
    );

    endpoints.addEndpoint(FLEXIBLE_ENDPOINT()
        .route("/pingI2C")
        .summary("Ping I2C device")
        .description("Check if an I2C device is responding")
        .params({
            REQUIRED_INT_PARAM("bus_id", "Bus ID"),
            REQUIRED_STR_PARAM("device_addr", "Device address (hex format)")
        })
        .responseType(JSON_RESPONSE)
        .handler([this](std::map<String, String>& params) {
            return handlePingDevice(params);
        })
    );

    endpoints.addEndpoint(FLEXIBLE_ENDPOINT()
        .route("/readI2CBytes")
        .summary("Read multiple bytes from device")
        .description("Read multiple bytes from a register on an I2C device")
        .params({
            REQUIRED_INT_PARAM("bus_id", "Bus ID"),
            REQUIRED_STR_PARAM("device_addr", "Device address (hex format)"),
            REQUIRED_STR_PARAM("reg_addr", "Register address (hex format)"),
            REQUIRED_INT_PARAM("length", "Number of bytes to read")
        })
        .responseType(JSON_RESPONSE)
        .handler([this](std::map<String, String>& params) {
            return handleReadBytes(params);
        })
    );

    endpoints.addEndpoint(FLEXIBLE_ENDPOINT()
        .route("/writeI2CBytes")
        .summary("Write multiple bytes to device")
        .description("Write multiple bytes to a register on an I2C device")
        .params({
            REQUIRED_INT_PARAM("bus_id", "Bus ID"),
            REQUIRED_STR_PARAM("device_addr", "Device address (hex format)"),
            REQUIRED_STR_PARAM("reg_addr", "Register address (hex format)"),
            REQUIRED_STR_PARAM("data", "Comma-separated hex values (e.g., '0x01,0x02,0x03')")
        })
        .responseType(JSON_RESPONSE)
        .handler([this](std::map<String, String>& params) {
            return handleWriteBytes(params);
        })
    );
}

std::pair<String, int> FlexibleI2C::handleInitBus(std::map<String, String>& params) {
    DynamicJsonDocument response(1024);

    if (params.find("bus_id") == params.end() || params.find("sda_pin") == params.end() || params.find("scl_pin") == params.end()) {
        response["success"] = false;
        response["error"] = "Missing required parameters";
        String output;
        serializeJson(response, output);
        return {output, 400};
    }

    uint8_t bus_id = params["bus_id"].toInt();
    uint8_t sda_pin = params["sda_pin"].toInt();
    uint8_t scl_pin = params["scl_pin"].toInt();
    uint32_t frequency = params.find("frequency") != params.end() ? params["frequency"].toInt() : 100000;

    bool success = initBus(bus_id, sda_pin, scl_pin, frequency);

    response["success"] = success;
    response["bus_id"] = bus_id;
    response["sda_pin"] = sda_pin;
    response["scl_pin"] = scl_pin;
    response["frequency"] = frequency;

    if (!success) {
        response["error"] = getErrorString(getLastError());
    }

    String output;
    serializeJson(response, output);
    return {output, success ? 200 : 500};
}

std::pair<String, int> FlexibleI2C::handleScanBus(std::map<String, String>& params) {
    DynamicJsonDocument response(1024);

    if (params.find("bus_id") == params.end()) {
        response["success"] = false;
        response["error"] = "Missing bus_id parameter";
        String output;
        serializeJson(response, output);
        return {output, 400};
    }

    uint8_t bus_id = params["bus_id"].toInt();
    std::vector<uint8_t> devices = scanBus(bus_id);

    response["success"] = (getLastError() == SUCCESS);
    response["bus_id"] = bus_id;
    response["device_count"] = devices.size();

    JsonArray device_array = response["devices"].to<JsonArray>();
    for (uint8_t addr : devices) {
        JsonObject device = device_array.add<JsonObject>();
        device["address"] = addr;
        device["address_hex"] = "0x" + String(addr, HEX);
    }

    if (getLastError() != SUCCESS) {
        response["error"] = getErrorString(getLastError());
    }

    String output;
    serializeJson(response, output);
    return {output, response["success"] ? 200 : 500};
}

std::pair<String, int> FlexibleI2C::handleDeviceInfo(std::map<String, String>& params) {
    DynamicJsonDocument response(1024);

    response["success"] = true;
    response["device_count"] = known_devices.size();

    JsonArray devices_array = response["devices"].to<JsonArray>();
    for (const auto& device : known_devices) {
        JsonObject device_obj = devices_array.add<JsonObject>();
        device_obj["bus_id"] = device.bus_id;
        device_obj["address"] = device.address;
        device_obj["address_hex"] = "0x" + String(device.address, HEX);
        device_obj["name"] = device.device_name;
        device_obj["responsive"] = device.responsive;
        device_obj["last_seen"] = device.last_seen;
    }

    String output;
    serializeJson(response, output);
    return {output, 200};
}

std::pair<String, int> FlexibleI2C::handleReadRegister(std::map<String, String>& params) {
    DynamicJsonDocument response(1024);

    if (params.find("bus_id") == params.end() || params.find("device_addr") == params.end() || params.find("reg_addr") == params.end()) {
        response["success"] = false;
        response["error"] = "Missing required parameters";
        String output;
        serializeJson(response, output);
        return {output, 400};
    }

    uint8_t bus_id = params["bus_id"].toInt();
    uint8_t device_addr = strtol(params["device_addr"].c_str(), NULL, 16);
    uint8_t reg_addr = strtol(params["reg_addr"].c_str(), NULL, 16);

    uint8_t value = readRegister(bus_id, device_addr, reg_addr);
    bool success = (getLastError() == SUCCESS);

    response["success"] = success;
    response["bus_id"] = bus_id;
    response["device_addr"] = "0x" + String(device_addr, HEX);
    response["reg_addr"] = "0x" + String(reg_addr, HEX);

    if (success) {
        response["value"] = value;
        response["value_hex"] = "0x" + String(value, HEX);
    } else {
        response["error"] = getErrorString(getLastError());
    }

    String output;
    serializeJson(response, output);
    return {output, success ? 200 : 500};
}

std::pair<String, int> FlexibleI2C::handleWriteRegister(std::map<String, String>& params) {
    DynamicJsonDocument response(1024);

    if (params.find("bus_id") == params.end() || params.find("device_addr") == params.end() ||
        params.find("reg_addr") == params.end() || params.find("value") == params.end()) {
        response["success"] = false;
        response["error"] = "Missing required parameters";
        String output;
        serializeJson(response, output);
        return {output, 400};
    }

    uint8_t bus_id = params["bus_id"].toInt();
    uint8_t device_addr = strtol(params["device_addr"].c_str(), NULL, 16);
    uint8_t reg_addr = strtol(params["reg_addr"].c_str(), NULL, 16);
    uint8_t value = strtol(params["value"].c_str(), NULL, 16);

    bool success = writeRegister(bus_id, device_addr, reg_addr, value);

    response["success"] = success;
    response["bus_id"] = bus_id;
    response["device_addr"] = "0x" + String(device_addr, HEX);
    response["reg_addr"] = "0x" + String(reg_addr, HEX);
    response["value"] = "0x" + String(value, HEX);

    if (!success) {
        response["error"] = getErrorString(getLastError());
    }

    String output;
    serializeJson(response, output);
    return {output, success ? 200 : 500};
}

std::pair<String, int> FlexibleI2C::handlePingDevice(std::map<String, String>& params) {
    DynamicJsonDocument response(1024);

    if (params.find("bus_id") == params.end() || params.find("device_addr") == params.end()) {
        response["success"] = false;
        response["error"] = "Missing required parameters";
        String output;
        serializeJson(response, output);
        return {output, 400};
    }

    uint8_t bus_id = params["bus_id"].toInt();
    uint8_t device_addr = strtol(params["device_addr"].c_str(), NULL, 16);

    bool present = isDevicePresent(bus_id, device_addr);

    response["success"] = true;
    response["bus_id"] = bus_id;
    response["device_addr"] = "0x" + String(device_addr, HEX);
    response["present"] = present;

    String output;
    serializeJson(response, output);
    return {output, 200};
}

std::pair<String, int> FlexibleI2C::handleReadBytes(std::map<String, String>& params) {
    DynamicJsonDocument response(1024);

    if (params.find("bus_id") == params.end() || params.find("device_addr") == params.end() ||
        params.find("reg_addr") == params.end() || params.find("length") == params.end()) {
        response["success"] = false;
        response["error"] = "Missing required parameters";
        String output;
        serializeJson(response, output);
        return {output, 400};
    }

    uint8_t bus_id = params["bus_id"].toInt();
    uint8_t device_addr = strtol(params["device_addr"].c_str(), NULL, 16);
    uint8_t reg_addr = strtol(params["reg_addr"].c_str(), NULL, 16);
    uint8_t length = params["length"].toInt();

    if (length > 64) { // Reasonable limit
        response["success"] = false;
        response["error"] = "Length too large (max 64 bytes)";
        String output;
        serializeJson(response, output);
        return {output, 400};
    }

    uint8_t data[64];
    bool success = readBytes(bus_id, device_addr, reg_addr, data, length);

    response["success"] = success;
    response["bus_id"] = bus_id;
    response["device_addr"] = "0x" + String(device_addr, HEX);
    response["reg_addr"] = "0x" + String(reg_addr, HEX);
    response["length"] = length;

    if (success) {
        JsonArray data_array = response["data"].to<JsonArray>();
        for (uint8_t i = 0; i < length; i++) {
            data_array.add("0x" + String(data[i], HEX));
        }
    } else {
        response["error"] = getErrorString(getLastError());
    }

    String output;
    serializeJson(response, output);
    return {output, success ? 200 : 500};
}

std::pair<String, int> FlexibleI2C::handleWriteBytes(std::map<String, String>& params) {
    DynamicJsonDocument response(1024);

    if (params.find("bus_id") == params.end() || params.find("device_addr") == params.end() ||
        params.find("reg_addr") == params.end() || params.find("data") == params.end()) {
        response["success"] = false;
        response["error"] = "Missing required parameters";
        String output;
        serializeJson(response, output);
        return {output, 400};
    }

    uint8_t bus_id = params["bus_id"].toInt();
    uint8_t device_addr = strtol(params["device_addr"].c_str(), NULL, 16);
    uint8_t reg_addr = strtol(params["reg_addr"].c_str(), NULL, 16);

    // Parse comma-separated hex values
    String data_str = params["data"];
    std::vector<uint8_t> data_bytes;

    int start = 0;
    int end = data_str.indexOf(',');

    while (end >= 0 || start < data_str.length()) {
        String byte_str = (end >= 0) ? data_str.substring(start, end) : data_str.substring(start);
        byte_str.trim();

        if (byte_str.length() > 0) {
            uint8_t byte_val = strtol(byte_str.c_str(), NULL, 16);
            data_bytes.push_back(byte_val);
        }

        if (end < 0) break;
        start = end + 1;
        end = data_str.indexOf(',', start);
    }

    if (data_bytes.empty()) {
        response["success"] = false;
        response["error"] = "No valid data bytes provided";
        String output;
        serializeJson(response, output);
        return {output, 400};
    }

    bool success = writeBytes(bus_id, device_addr, reg_addr, data_bytes.data(), data_bytes.size());

    response["success"] = success;
    response["bus_id"] = bus_id;
    response["device_addr"] = "0x" + String(device_addr, HEX);
    response["reg_addr"] = "0x" + String(reg_addr, HEX);
    response["bytes_written"] = data_bytes.size();

    if (!success) {
        response["error"] = getErrorString(getLastError());
    }

    String output;
    serializeJson(response, output);
    return {output, success ? 200 : 500};
}

DynamicJsonDocument FlexibleI2C::deviceInfoToJson(const I2CDeviceInfo& device) {
    DynamicJsonDocument doc(512);
    doc["bus_id"] = device.bus_id;
    doc["address"] = device.address;
    doc["address_hex"] = "0x" + String(device.address, HEX);
    doc["name"] = device.device_name;
    doc["responsive"] = device.responsive;
    doc["last_seen"] = device.last_seen;
    return doc;
}

DynamicJsonDocument FlexibleI2C::busConfigToJson(uint8_t bus_id) {
    DynamicJsonDocument doc(512);
    auto it = buses.find(bus_id);
    if (it != buses.end()) {
        doc["bus_id"] = bus_id;
        doc["sda_pin"] = it->second.sda_pin;
        doc["scl_pin"] = it->second.scl_pin;
        doc["frequency"] = it->second.frequency;
        doc["initialized"] = it->second.initialized;
    }
    return doc;
}