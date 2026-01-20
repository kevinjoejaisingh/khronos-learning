#include <iostream>
#include <string>

class Animal {
public:
    std::string name;
    
    Animal(std::string n) {
        name = n;
        std::cout << "Animal constructor: " << name << std::endl;
    }
    
    void eat() {
        std::cout << name << " is eating." << std::endl;
    }
};

class Dog : public Animal {
public:
    std::string breed;
    
    Dog(std::string n, std::string b) : Animal(n) {
        breed = b;
        std::cout << "Dog constructor: " << breed << std::endl;
    }
    
    void bark() {
        std::cout << name << " the " << breed << " says woof!" << std::endl;
    }
};

int main() {
    Dog myDog("Buddy", "Golden Retriever");
    myDog.eat();
    myDog.bark();
    
    return 0;
}