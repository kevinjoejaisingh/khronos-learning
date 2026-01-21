#include "depth_sensor.h"
#include <iostream>

depth_sensor::depth_sensor(std::string n) : Sensor(n) {
}

void depth_sensor::read() {
    std::cout << name << ": depth = 1.5m" << std::endl;
}