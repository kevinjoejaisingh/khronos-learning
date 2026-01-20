#include <iostream>

int add(int a, int b) {
    return a + b;
}

void greet(std::string name) {
    std::cout << "Hello, " << name << "!" << std::endl;
}

int main() {
    int result = add(5, 3);
    std::cout << "5 + 3 = " << result << std::endl;

    greet("Kevin");

    return 0;
}