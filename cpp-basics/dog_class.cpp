#include <iostream>
#include "dog.h"

Dog::Dog(std::string name, int age) {
    this->name = name;
    this->age = age;
}

void Dog::bark() {
    std::cout << name << " says woof!" << std::endl;
}

int Dog::getAge() { 
    return age;
}

std::string Dog::getName() {
    return name;
}

Dog::~Dog() {
    std::cout << name << " is being destroyed!" << std::endl;
}
