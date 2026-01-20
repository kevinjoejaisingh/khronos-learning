#include <iostream>
#include <memory>
#include <string>

class sensor {
    private :
        std::string name;
        double reading;

    public :
        sensor(std::string name) {
            this->name = name;
            this->reading = 0.0;
        }

        void takeMeasurement(double newReading) {
            reading = newReading;
        }

        double getReading() {
            return reading;
        }

        std::string getName() {
            return name;
        }

        ~sensor() {
            std::cout << "Sensor " << name << " is shutting down!" << std::endl;
        }
    };
        void processSensor(sensor& s) {
            s.takeMeasurement(42.5); 
            std::cout << "Processed sensor " << s.getName() << ", new reading: " << s.getReading() << std::endl;
        }


int main() {
        std::unique_ptr<sensor> D455 = std::make_unique<sensor>("D455");
        processSensor(*D455);
        std::shared_ptr<sensor> Lidar = std::make_shared<sensor>("Lidar");
        std::shared_ptr<sensor> Lidar2 = Lidar;  
        std::cout << "Reference count: " << Lidar.use_count() << std::endl;







}