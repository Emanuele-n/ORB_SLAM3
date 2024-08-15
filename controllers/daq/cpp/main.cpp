#include "USB1408FS.hpp"

int main(int argc, char **argv)
{
    USB1408FS daqSystem;
    
    if (daqSystem.findDevice()) {
        daqSystem.initialize();
        daqSystem.configurePorts();
        while (true) {
            daqSystem.readInputs();
        }
    }
    return 0;
}
