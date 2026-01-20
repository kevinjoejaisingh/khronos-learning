#ifndef DOG_H
#define DOG_H

#include <string>

class Dog {
private:
    std::string name;
    int age;

public:
    Dog(std::string n, int a);
    void bark();
    int getAge();
    std::string getName();
    ~Dog();
};

#endif