# Khronos Learning Journey

My path to understanding C++, ROS2, and the MIT SPARK stack for drone SLAM.

---

## Progress

| Phase | Topic | Status |
|-------|-------|--------|
| 1 | C++ Fundamentals | ✅ |
| 2 | CMake | ✅ |
| 3 | Math & Coordinate Frames | ✅ |
| 4 | ROS2 Fundamentals | 🔜 |
| 5 | D455 Integration | ⬜ |
| 6 | GTSAM & Factor Graphs | ⬜ |
| 7 | Kimera-VIO | ⬜ |
| 8 | Kimera-Semantics | ⬜ |
| 9 | Hydra | ⬜ |
| 10 | Khronos | ⬜ |

---

## C++ Cheat Sheet

### Basics
```cpp
// Variables - must declare type
int x = 5;
double price = 19.99;
char grade = 'A';           // single quotes
std::string name = "Kevin"; // double quotes
bool is_ready = true;       // prints as 1/0

// Integer division truncates
10 / 3  // = 3, not 3.33
10 % 3  // = 1 (modulo/remainder)

// Control flow
if (x > 5) {
    // code
} else if (x > 0) {
    // code
} else {
    // code
}

// Loops
for (int i = 0; i < 10; i++) { }
while (condition) { }
for (int num : vec) { }  // range-based
```

### Functions
```cpp
// Must declare return type and parameter types
int add(int a, int b) {
    return a + b;
}

void greet(std::string name) {  // void = no return
    std::cout << "Hello " << name << std::endl;
}

// Function prototype (declare before define)
int add(int a, int b);  // declaration
// ... later ...
int add(int a, int b) { return a + b; }  // definition
```

### Classes
```cpp
class Dog {
private:
    std::string name;  // only accessible inside class
    
public:
    // Constructor - same name as class, no return type
    Dog(std::string n) {
        name = n;
    }
    
    // Or with initializer list
    Dog(std::string n) : name(n) {}
    
    // Destructor - cleanup when object dies
    ~Dog() {
        std::cout << name << " destroyed" << std::endl;
    }
    
    // Methods
    void bark() {
        std::cout << name << " says woof!" << std::endl;
    }
    
    // Getter
    std::string getName() { return name; }
};

// Using the class
Dog myDog("Buddy");
myDog.bark();
```

### Header/Source File Split
```cpp
// dog.h - declarations
#ifndef DOG_H
#define DOG_H

class Dog {
public:
    Dog(std::string n);
    void bark();
private:
    std::string name;
};

#endif

// dog.cpp - definitions
#include "dog.h"

Dog::Dog(std::string n) : name(n) {}

void Dog::bark() {
    std::cout << name << " says woof!" << std::endl;
}
```

### Pointers & References
```cpp
int x = 42;

// Pointer - holds memory address
int* ptr = &x;      // & = "address of"
*ptr = 100;         // * = "value at address" (dereference)
// x is now 100

// Reference - alias for existing variable
int& ref = x;       // ref IS x
ref = 50;           // x is now 50

// Pass by value - copy, original unchanged
void func(int n) { n = 999; }

// Pass by reference - original changes
void func(int& n) { n = 999; }

// Const reference - read only, no copy
void func(const int& n) { /* can't modify n */ }

// Heap allocation
int* heapVar = new int;
*heapVar = 42;
delete heapVar;     // MUST free memory

// Pointers to objects use ->
Dog* ptr = new Dog("Max");
ptr->bark();        // arrow operator
delete ptr;
```

### Smart Pointers (Modern C++)
```cpp
#include <memory>

// unique_ptr - single owner, auto-deletes
std::unique_ptr<Dog> dog1 = std::make_unique<Dog>("Buddy");
dog1->bark();
// auto-deleted when out of scope

// shared_ptr - multiple owners, auto-deletes when last one dies
std::shared_ptr<Dog> dog2 = std::make_shared<Dog>("Max");
std::shared_ptr<Dog> dog3 = dog2;  // both point to same dog
dog2.use_count();  // = 2
// deleted when both go out of scope
```

### STL Containers
```cpp
#include <vector>
#include <map>

// Vector - dynamic array
std::vector<int> nums;
nums.push_back(10);      // add to end
nums.pop_back();         // remove from end
nums[0];                 // access by index
nums.size();             // count
nums.front();            // first element
nums.back();             // last element

std::vector<int> nums = {1, 2, 3};  // initialize with values

// Loop through vector
for (int& num : nums) {
    std::cout << num << std::endl;
}

// Map - key/value pairs
std::map<std::string, int> ages;
ages["Alice"] = 25;
ages["Bob"] = 30;

// Check if key exists (don't use [] for checking!)
if (ages.find("Charlie") == ages.end()) {
    std::cout << "Not found" << std::endl;
}

// Loop through map
for (auto& pair : ages) {
    std::cout << pair.first << ": " << pair.second << std::endl;
}
```

### Templates
```cpp
// Function template - works with any type
template <typename T>
T maxValue(T a, T b) {
    return (a > b) ? a : b;
}

maxValue(5, 10);        // T is int
maxValue(3.14, 2.71);   // T is double

// Class template
template <typename T>
class Box {
private:
    T contents;
public:
    Box(T item) : contents(item) {}
    T get() { return contents; }
};

Box<int> intBox(42);
Box<std::string> strBox("Hello");
```

### Inheritance & Polymorphism
```cpp
class Animal {
protected:
    std::string name;
public:
    Animal(std::string n) : name(n) {}
    
    virtual ~Animal() {}  // ALWAYS virtual destructor if inherited
    
    virtual void speak() {  // virtual = can be overridden
        std::cout << "Some sound" << std::endl;
    }
};

class Dog : public Animal {  // Dog inherits from Animal
public:
    Dog(std::string n) : Animal(n) {}  // call parent constructor
    
    void speak() override {  // override parent method
        std::cout << name << " says woof!" << std::endl;
    }
};

// Polymorphism - base pointer, derived behavior
Animal* ptr = new Dog("Buddy");
ptr->speak();  // prints "Buddy says woof!" (because virtual)
delete ptr;    // calls Dog destructor (because virtual destructor)
```

---

## CMake Cheat Sheet

### Basic Structure
```cmake
cmake_minimum_required(VERSION 3.10)
project(MyProject)

# Set C++ standard
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# Simple executable
add_executable(my_program main.cpp)

# Multiple source files
add_executable(my_program main.cpp utils.cpp helper.cpp)
```

### Libraries
```cmake
# Create a library
add_library(my_lib source1.cpp source2.cpp)

# Create executable
add_executable(my_program main.cpp)

# Link library to executable
target_link_libraries(my_program my_lib)
```

### Finding External Libraries
```cmake
# Find installed packages (like ROS2)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)

# Link them
target_link_libraries(my_program ${rclcpp_LIBRARIES})
```

### Build Commands
```bash
# Create build directory (out-of-source build)
mkdir build
cd build

# Generate build files
cmake ..

# Build the project
make

# Run executable
./my_program
```

### Directory Structure
```
my_project/
├── CMakeLists.txt
├── include/
│   └── my_header.h
├── src/
│   ├── main.cpp
│   └── utils.cpp
└── build/          # generated, don't commit
    └── ...
```

---

## Math & Coordinate Frames Cheat Sheet

### Rotations

| Name | Axis | What it does |
|------|------|--------------|
| Roll | X | Tilt sideways |
| Pitch | Y | Tilt forward/back |
| Yaw | Z | Spin left/right |

### Euler Angles vs Quaternions

| Euler Angles | Quaternions |
|--------------|-------------|
| 3 values (roll, pitch, yaw) | 4 values (w, x, y, z) |
| Human readable | Not human readable |
| Gimbal lock problem | No gimbal lock |
| Bad for computation | Good for computation |

### Transforms
```
Transform = Rotation + Translation

4x4 Homogeneous Matrix:
[R  R  R | tx]
[R  R  R | ty]
[R  R  R | tz]
[0  0  0 | 1 ]

To transform point from frame A to frame B:
P_B = T_BA × P_A
```

### TF Tree
```
world
  └── drone
        ├── camera
        ├── imu
        └── lidar

Each frame has ONE parent.
Chain transforms to go between frames.
```