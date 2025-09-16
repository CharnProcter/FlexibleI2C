# FlexibleI2C

A flexible I2C library for ESP32 supporting up to 2 buses with extensible HTTP endpoint interface built on FlexibleEndpoints.

## Features

- Support for up to 2 I2C buses (Wire and Wire1)
- Configurable SDA, SCL pins and frequencies
- Device scanning and management
- Comprehensive I2C operations (register read/write, multi-byte operations)
- HTTP API endpoints via FlexibleEndpoints integration
- Extensible architecture for building specialized device controllers
- JSON responses for all operations
- Error handling and device status tracking

## HTTP Endpoints

- `POST /initI2C` - Initialize I2C bus
- `GET /scanI2C?bus_id=0` - Scan bus for devices
- `GET /getI2CDevices` - List all known devices
- `GET /readI2C?bus_id=0&device_addr=0x48&reg_addr=0x00` - Read register
- `POST /writeI2C` - Write register
- `GET /pingI2C?bus_id=0&device_addr=0x48` - Ping device
- `GET /readI2CBytes` - Read multiple bytes
- `POST /writeI2CBytes` - Write multiple bytes

## Usage

```cpp
#include <FlexibleI2C.h>
#include <FlexibleEndpoints.h>

FlexibleI2C i2c;
// Initialize with FlexibleEndpoints instance
i2c.init(endpoints);

// Initialize I2C bus
i2c.initBus(0, 21, 22, 100000); // bus_id, sda, scl, frequency

// Scan for devices
std::vector<uint8_t> devices = i2c.scanBus(0);

// Read/write operations
uint8_t value = i2c.readRegister(0, 0x48, 0x00);
i2c.writeRegister(0, 0x48, 0x00, 0xFF);
```

## Extending FlexibleI2C

Create specialized device controllers by inheriting from FlexibleI2C:

```cpp
class MyDeviceController : public FlexibleI2C {
public:
    virtual void registerCustomEndpoints(FlexibleEndpoints& endpoints) override {
        // Add device-specific endpoints
    }

    virtual void onDeviceFound(uint8_t bus_id, uint8_t address) override {
        // Handle device discovery
    }
};
```

## License

MIT License