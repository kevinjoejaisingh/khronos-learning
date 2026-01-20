#include <iostream>
#include "dog.h"

int main() {
    Dog myDog("Buddy", 3);

    std::cout << myDog.getName() << " is " << myDog.getAge() << " years old." << std::endl;
    myDog.bark();

    return 0;
}