#include <iostream>

int main() {
    int x = 42;
    int& ref = x;  // ref is a reference to x

    std::cout << "x: " << x << std::endl;
    std::cout << "ref: " << ref << std::endl;

    ref = 100;

    std::cout << "\nAfter ref = 100:" << std::endl;
    std::cout << "x: " << x << std::endl;
    std::cout << "ref: " << ref << std::endl;

    return 0;
}