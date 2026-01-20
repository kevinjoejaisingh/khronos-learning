#include <iostream>
#include <map>
#include <string>

int main() {
    std::map<std::string, int> ages;
    
    ages["Alice"] = 25;
    ages["Bob"] = 30;
    ages["Charlie"] = 22;
    
    std::cout << "Alice's age: " << ages["Alice"] << std::endl;
    std::cout << "Map size: " << ages.size() << std::endl;
    
    std::cout << "\nAll entries:" << std::endl;
    for (auto& pair : ages) {
        std::cout << pair.first << " -> " << pair.second << std::endl;
    }
    
    // Checking if key exists
    if (ages.find("David") == ages.end()) {
        std::cout << "\nDavid not found!" << std::endl;
    }
    
    if (ages.find("Alice") != ages.end()) {
        std::cout << "Alice found!" << std::endl;
    }
    return 0;
}