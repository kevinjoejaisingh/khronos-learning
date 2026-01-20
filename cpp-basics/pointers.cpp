#include <iostream>

int main() {
    int x = 42;
    int* ptr = &x;

    std::cout << "Value of x: " << x << std::endl;
    std::cout << "Address of x: " << &x << std::endl;
    std::cout << "ptr holds: " << ptr << std::endl;
    std::cout << "Value at *ptr: " << *ptr << std::endl;
    *ptr = 100;
    std::cout << "\nAfter *ptr = 100:" << std::endl;
    std::cout << "Value of x: " << x << std::endl;

    return 0;
}