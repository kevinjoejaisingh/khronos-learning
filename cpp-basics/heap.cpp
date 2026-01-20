#include <iostream>

int main() {
    // Stack allocation
    int stackVar = 10;

    // Heap allocation
    int* heapVar = new int;
    *heapVar = 20;

    std::cout << "stackVar: " << stackVar << std::endl;
    std::cout << "heapVar points to: " << *heapVar << std::endl;

    // MUST free heap memory when done
    delete heapVar;

    return 0;
}