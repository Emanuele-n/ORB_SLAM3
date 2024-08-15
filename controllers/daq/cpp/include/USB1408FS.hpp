#ifndef USB1408FS_HPP
#define USB1408FS_HPP

#include <libusb-1.0/libusb.h>
#include <stdint.h>

class USB1408FS {
public:
    USB1408FS();
    ~USB1408FS();
    bool findDevice();
    void initialize();
    void configurePorts();
    void readInputs();

private:
    libusb_device_handle *udev;
    bool deviceFound;
    void checkLibUSBError(int result, const char* context);
    float convertToVoltage(int digitalValue, uint8_t gain) const;
    float convertToPsi(float voltage) const;
};

#endif // USB1408FS_HPP
