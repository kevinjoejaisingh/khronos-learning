#include <iostream>
#include <string>

bool is_even(int number) {
    return number % 2 == 0;
}

void fizzbuzz(int n) {
    if (n % 3 == 0 && n % 5 == 0){
        std::cout << "FizzBuzz";
    }
    else if (n % 5 == 0){
        std::cout << "Buzz";
    }
    else if (n % 3 == 0){
        std::cout << "Fizz";
    }
    else {
        std::cout << n;
    }
}

int main() {
    for (int i = 1; i <= 15; i++) {
        if (is_even(i)) {
            std::cout << i << " is even. ";
        } else {
            std::cout << i << " is odd. ";
        }
        fizzbuzz(i);
        std::cout << std::endl;
        std::cout << "\n" << std::endl;
    }
}