#include <iostream>
#include <memory>
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
    std::cout << "--- unique_ptr ---" << std::endl;
    std::unique_ptr<Dog> dog1 = std::make_unique<Dog>("Buddy");
    dog1->bark();

    std::cout << "\n--- End of main ---" << std::endl;

    std::cout << "\n--- shared_ptr ---" << std::endl;
    std::shared_ptr<Dog> dog2 = std::make_shared<Dog>("Max");
    std::shared_ptr<Dog> dog3 = dog2;  // Both point to same dog
    
    std::cout << "dog2 name: " << dog2->name << std::endl;
    std::cout << "dog3 name: " << dog3->name << std::endl;
    std::cout << "Reference count: " << dog2.use_count() << std::endl;
    return 0;

}