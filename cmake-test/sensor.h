#ifndef SENSOR_H
#define SENSOR_H

#include <string>

class Sensor {
protected:
    std::string name;

public:
    Sensor(std::string n);
    virtual ~Sensor();
    virtual void read();
};

#endif