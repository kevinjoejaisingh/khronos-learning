#ifndef DEPTH_H
#define DEPTH_H
#include "sensor.h"


class depth_sensor : public Sensor
{

public:
    depth_sensor(std::string n);
    void read() override;
    
};







#endif