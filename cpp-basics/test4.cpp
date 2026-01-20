#include <iostream>
#include <string>
#include <map>
#include <vector>

template <typename T>
class DataBuffer {
    private:
        std::vector<T> data;
    
    public:
    void add(T item) {
        data.push_back(item);
    }

    T getLatest(){
        if (!data.empty()) {
            return data.back();
        }
        throw std::out_of_range("DataBuffer is empty");
    }

    int getSize(){
        return data.size();
    }

    std::vector<T> getAll(){
        return data;
    }

};


    int main() {

        std::map<std::string, double> sensorReadings;
        sensorReadings["offset"] = 0.5;
        sensorReadings["scale"] = 1.2;
        sensorReadings["bias"] = 0.1;

        DataBuffer<double> buffer;
        buffer.add(1.5);
        buffer.add(2.3);
        buffer.add(1.8);
        std::cout << "Buffer size: " << buffer.getSize() << std::endl;
        std::cout << "Latest reading: " << buffer.getLatest() << std::endl;

        for (const auto& pair : sensorReadings) {
            std::cout << pair.first << ": " << pair.second << std::endl;
        }   

        if (sensorReadings.find("gain") == sensorReadings.end()) {
            std::cout << "\ngain not found!" << std::endl;
    }
        for (auto& buff: buffer.getAll()) {
            std::cout << "Buffered value: " << buff << std::endl;
        }

    }