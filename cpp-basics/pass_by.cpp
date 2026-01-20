#include <iostream>

void passByValue(int n) {
    n = 999;
}

void passByReference(int& n) {
    n = 999;
}

int main() {
    int a = 10;
    int b = 10;

    passByValue(a);
    passByReference(b);

    std::cout << "a after passByValue: " << a << std::endl;
    std::cout << "b after passByReference: " << b << std::endl;

    return 0;
}