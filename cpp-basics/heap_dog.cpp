#include <iostream>
#include <string>

class Dog {
public:
    std::string name;
    
    Dog(std::string n) {
        name = n;
        std::cout << name << " is born!" << std::endl;
    }
    
    ~Dog() {
        std::cout << name << " is destroyed!" << std::endl;
    }
    
    void bark() {
        std::cout << name << " says woof!" << std::endl;
    }
};

int main() {
    std::cout << "--- Stack dog ---" << std::endl;
    Dog stackDog("Buddy");
    stackDog.bark();

    std::cout << "\n--- Heap dog ---" << std::endl;
    Dog* heapDog = new Dog("Max");
    heapDog->bark();

    std::cout << "\n--- Deleting heap dog ---" << std::endl;
    delete heapDog;

    std::cout << "\n--- End of main ---" << std::endl;
    return 0;
}