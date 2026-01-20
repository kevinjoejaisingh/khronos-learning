#include <iostream>
#include <string>
#include <vector>


class Sensor{

    private :
        std::string name;

    public :
    Sensor(std::string name) {
        this->name = name;
    }

    virtual ~Sensor() {
        std::cout << "Sensor " << name << " is shutting down!" << std::endl;
    }

    virtual void readData(){
        std::cout << "Reading data from sensor " << name << std::endl;
    }

    std::string getName() {
        return name;
    }
};
 

class DepthSensor : public Sensor {
    public:
    DepthSensor(std::string name) : Sensor(name) {}

    void readData() override {
        std::cout << "Reading depth data from sensor " << getName() << std::endl;
    }


};

class IMUSensor : public Sensor {
    public:
    IMUSensor(std::string name) : Sensor(name) {}

    void readData() override {
        std::cout << "Reading IMU data from sensor " << getName() << std::endl;
    }

};

    int main() {
        std::vector<Sensor*> sensors;
        sensors.push_back(new DepthSensor("D455"));
        sensors.push_back(new IMUSensor("BMI088"));
        for (auto& sen: sensors){
            sen->readData();
        }
       
        for (auto& sen: sensors){
            delete sen;
        }

        return 0;
    }

