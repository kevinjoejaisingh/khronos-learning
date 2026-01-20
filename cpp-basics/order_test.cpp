#include <iostream>

int add(int a, int b);

int main() {
    int result = add(5, 3);
    std::cout << result << std::endl;
    return 0;
}

int add(int a, int b) {
    return a + b;
}