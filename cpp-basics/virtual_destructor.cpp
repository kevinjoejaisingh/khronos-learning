#include <iostream>
#include <string>

class Animal {
public:
    std::string name;
    
    Animal(std::string n) : name(n) {
        std::cout << "Animal " << name << " born" << std::endl;
    }

    virtual ~Animal() {
        std::cout << "Animal " << name << " destroyed" << std::endl;
    }
};
 
class Dog : public Animal {
public:
    Dog(std::string n) : Animal(n) {
        std::cout << "Dog " << name << " born" << std::endl;
    }
    
    ~Dog() {
        std::cout << "Dog " << name << " destroyed" << std::endl;
    }
};

int main() {
    std::cout << "--- Deleting through base pointer ---" << std::endl;
    Animal* ptr = new Dog("Buddy");
    delete ptr;
    
    return 0;
}