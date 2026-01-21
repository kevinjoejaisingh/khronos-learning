#include "sensor.h"
#include <iostream>

    Sensor::Sensor(std::string name) {
        this->name = name;
    }
    
    Sensor::~Sensor() {
        std::cout << "Sensor " << name << " is shutting down!" << std::endl;
    }
    void Sensor::read(){
        std::cout << "Reading data from sensor " << name << std::endl;
    }