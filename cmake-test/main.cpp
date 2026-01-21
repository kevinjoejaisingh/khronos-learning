#include "depth_sensor.h"
#include <iostream>
#include "sensor.h"
#include <vector>

int main() {
    std::vector<Sensor*> sensorpos;
    sensorpos.push_back(new depth_sensor("D455"));

    for (auto sensor : sensorpos) {
        sensor->read();
        delete sensor;
    }

    return 0;
}