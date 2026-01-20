#include <iostream>
#include <string>

class Animal {
public:
    std::string name;
    
    Animal(std::string n) : name(n) {}
    
    virtual void speak() {
        std::cout << name << " makes a sound." << std::endl;
    }
};

class Dog : public Animal {
public:
    Dog(std::string n) : Animal(n) {}
    
    void speak() {
        std::cout << name << " says woof!" << std::endl;
    }
};

class Cat : public Animal {
public:
    Cat(std::string n) : Animal(n) {}
    
    void speak() {
        std::cout << name << " says meow!" << std::endl;
    }
};

int main() {
    Animal generic("Generic");
    Dog dog("Buddy");
    Cat cat("Whiskers");
    
    generic.speak();
    dog.speak();
    cat.speak();
    
    std::cout << "\n--- Using pointers ---" << std::endl;
    Animal* ptr;
    
    ptr = &dog;
    ptr->speak();
    
    ptr = &cat;
    ptr->speak();

    return 0;
}   