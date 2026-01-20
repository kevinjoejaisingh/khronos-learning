#include <iostream>

int main() {
    // For loop
    std::cout << "For loop:" << std::endl;
    for (int i = 0; i < 5; i++) {
        std::cout << "i = " << i << std::endl;
    }

    // While loop
    std::cout << "\nWhile loop:" << std::endl;
    int count = 0;
    while (count < 5) {
        std::cout << "count = " << count << std::endl;
        count++;
    }

    return 0;
}