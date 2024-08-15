#include "USB1408FS.hpp"
#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <fcntl.h>
#include <iostream>
#include "pmd.h"
#include "usb-1408FS.h"

USB1408FS::USB1408FS() : udev(nullptr), deviceFound(false) {
    int result = libusb_init(NULL);
    checkLibUSBError(result, "Initialize libusb");
}

USB1408FS::~USB1408FS() {
    if (udev) {
        libusb_close(udev);
    }
    libusb_exit(NULL);
}

bool USB1408FS::findDevice() {
    udev = usb_device_find_USB_MCC(USB1408FS_PID, NULL);
    deviceFound = (udev != nullptr);
    if (deviceFound) {
        std::cout << "USB-1408FS Device is found!\n";
    } else {
        std::cout << "No device found.\n";
    }
    return deviceFound;
}

void USB1408FS::initialize() {
    if (deviceFound) {
        init_USB1408FS(udev);
    }
}

void USB1408FS::configurePorts() {
    usbDConfigPort_USB1408FS(udev, DIO_PORTA, DIO_DIR_OUT);
    usbDConfigPort_USB1408FS(udev, DIO_PORTB, DIO_DIR_IN);
    usbDOut_USB1408FS(udev, DIO_PORTA, 0);
}

void USB1408FS::readInputs() {
    uint8_t gain = SE_10_00V;
    int flag = fcntl(fileno(stdin), F_GETFL);
    fcntl(0, F_SETFL, flag | O_NONBLOCK);

    for (uint8_t channel = 1; channel <= 4; channel++) {
        signed short svalue = usbAIn_USB1408FS(udev, channel - 1, gain);
        std::cout << "Channel " << int(channel) << " voltage=" << convertToVoltage(svalue, gain) << "V" << " pressure=" << convertToPsi(convertToVoltage(svalue, gain)) << "psi" << std::endl;
        sleep(1);
    }

    fcntl(fileno(stdin), F_SETFL, flag);
}

void USB1408FS::checkLibUSBError(int result, const char* context) {
    if (result < 0) {
        std::cerr << context << ": " << libusb_error_name(result) << std::endl;
        exit(EXIT_FAILURE);
    }
}

float USB1408FS::convertToVoltage(int digitalValue, uint8_t gain) const {
    return volts_1408FS_SE(digitalValue);
}

float USB1408FS::convertToPsi(float voltage) const {
    float nullVoltage = 0.50; 
    float sensitivity = 0.016; // TODO: check sensitivity value

    // Subtract the null voltage
    float activeVoltage = voltage - nullVoltage;

    // Convert active voltage to pressure in psi
    float pressurePsi = activeVoltage / sensitivity;

    return pressurePsi;
}
