#include <iostream>
#include <vector>
#include <string>

class Reading {
public:
    double depth;
    double confidence;
    
    Reading(double d, double c) {
        depth = d;
        confidence = c;
    }
};

int main() {
    std::vector<Reading> readings;
    
    readings.push_back(Reading(1.5, 0.95));
    readings.push_back(Reading(2.3, 0.87));
    readings.push_back(Reading(1.8, 0.92));
    
    std::cout << "Total readings: " << readings.size() << std::endl;
    
    for (Reading& r : readings) {
        std::cout << "Depth: " << r.depth << ", Confidence: " << r.confidence << std::endl;
    }
    
    return 0;
}

