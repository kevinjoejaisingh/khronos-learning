#include <iostream>
#include <string>

template <typename T>
T maxValue(T a, T b) {
    return (a > b) ? a : b;
}

int main() {
    std::cout << "Max of 5, 10: " << maxValue(5, 10) << std::endl;
    std::cout << "Max of 3.14, 2.71: " << maxValue(3.14, 2.71) << std::endl;
    std::cout << "Max of 'a', 'z': " << maxValue('a', 'z') << std::endl;
    
    return 0;
}