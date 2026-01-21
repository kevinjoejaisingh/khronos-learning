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
---

## ROS2 Cheat Sheet

### Core Concepts
```
┌──────────┐     /topic      ┌──────────┐
│  Node A  │ ───────────────►│  Node B  │
│  (pub)   │    messages     │  (sub)   │
└──────────┘                 └──────────┘
```

| Concept | Description |
|---------|-------------|
| **Node** | Independent program that does one thing |
| **Topic** | Named channel for continuous data (like a radio frequency) |
| **Message** | Structured data sent on topics |
| **Publisher** | Sends data to a topic |
| **Subscriber** | Receives data from a topic |
| **Service** | Request/response (like a function call) |
| **Parameter** | Runtime configuration value |
| **Bag** | Recorded topic data for playback |
| **tf2** | Coordinate frame transform system |

### Command Line Tools
```bash
# Nodes
ros2 node list                    # List running nodes
ros2 node info /node_name         # Info about a node

# Topics
ros2 topic list                   # List all topics
ros2 topic echo /topic_name       # Listen to a topic
ros2 topic info /topic_name       # Show topic type and pub/sub count
ros2 topic hz /topic_name         # Show publish rate

# Messages
ros2 interface show std_msgs/msg/String    # Show message structure
ros2 interface show sensor_msgs/msg/Imu    # Show IMU message

# Services
ros2 service list                 # List all services
ros2 service call /srv_name type "{arg: value}"  # Call a service

# Parameters
ros2 param list /node_name        # List node's parameters
ros2 param get /node_name param   # Get parameter value
ros2 param set /node_name param value  # Set parameter

# Transforms (tf2)
ros2 run tf2_ros tf2_echo frame1 frame2   # Show transform between frames
ros2 run tf2_ros static_transform_publisher --x 1.0 --y 0 --z 0.5 --frame-id parent --child-frame-id child

# Bags
ros2 bag record /topic1 /topic2 -o bagname   # Record topics
ros2 bag play bagname                         # Playback recording
ros2 bag info bagname                         # Show bag info

# Running nodes
ros2 run package_name executable_name         # Run a node
ros2 launch package_name launch_file.py       # Run launch file
```

### Writing a Publisher Node
```cpp
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"

class MyPublisher : public rclcpp::Node {
public:
    MyPublisher() : Node("my_publisher") {
        // Create publisher on topic "my_topic" with queue size 10
        publisher_ = this->create_publisher<std_msgs::msg::String>("my_topic", 10);
        
        // Create timer that calls callback every 500ms
        timer_ = this->create_wall_timer(
            std::chrono::milliseconds(500),
            std::bind(&MyPublisher::timer_callback, this)
        );
    }

private:
    void timer_callback() {
        auto message = std_msgs::msg::String();
        message.data = "Hello World";
        publisher_->publish(message);
        RCLCPP_INFO(this->get_logger(), "Published: '%s'", message.data.c_str());
    }

    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr publisher_;
    rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char* argv[]) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<MyPublisher>());
    rclcpp::shutdown();
    return 0;
}
```

### Writing a Subscriber Node
```cpp
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"

class MySubscriber : public rclcpp::Node {
public:
    MySubscriber() : Node("my_subscriber") {
        subscription_ = this->create_subscription<std_msgs::msg::String>(
            "my_topic", 10,
            std::bind(&MySubscriber::topic_callback, this, std::placeholders::_1)
        );
    }

private:
    void topic_callback(const std_msgs::msg::String::SharedPtr msg) {
        RCLCPP_INFO(this->get_logger(), "Heard: '%s'", msg->data.c_str());
    }

    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr subscription_;
};

int main(int argc, char* argv[]) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<MySubscriber>());
    rclcpp::shutdown();
    return 0;
}
```

### Parameters
```cpp
// In constructor
this->declare_parameter("my_param", 100);           // Declare with default
int value = this->get_parameter("my_param").as_int();  // Get value

// Run with custom parameter
// ros2 run my_pkg my_node --ros-args -p my_param:=200
```

### CMakeLists.txt for ROS2
```cmake
cmake_minimum_required(VERSION 3.8)
project(my_package)

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)

# Create executable
add_executable(my_node src/my_node.cpp)
ament_target_dependencies(my_node rclcpp std_msgs)

# Install executable
install(TARGETS my_node
  DESTINATION lib/${PROJECT_NAME}
)

# Install launch files
install(DIRECTORY launch/
  DESTINATION share/${PROJECT_NAME}/launch
)

ament_package()
```

### package.xml
```xml
<?xml version="1.0"?>
<package format="3">
  <name>my_package</name>
  <version>0.0.1</version>
  <description>My ROS2 package</description>
  <maintainer email="you@email.com">Your Name</maintainer>
  <license>MIT</license>

  <buildtool_depend>ament_cmake</buildtool_depend>
  <depend>rclcpp</depend>
  <depend>std_msgs</depend>

  <export>
    <build_type>ament_cmake</build_type>
  </export>
</package>
```

### Launch File (Python)
```python
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='my_package',
            executable='my_publisher',
            name='publisher',
            parameters=[{'my_param': 100}]
        ),
        Node(
            package='my_package',
            executable='my_subscriber',
            name='subscriber'
        )
    ])
```

### Build and Run
```bash
# Create workspace
mkdir -p ~/ros2_ws/src
cd ~/ros2_ws/src

# Create package
ros2 pkg create --build-type ament_cmake my_package

# Build
cd ~/ros2_ws
colcon build

# Source (do this in every new terminal)
source install/setup.bash

# Run node
ros2 run my_package my_node

# Run launch file
ros2 launch my_package my_launch.py
```

### tf2 Transforms
```
TF Tree Example:
world
  └── drone
        ├── camera
        ├── imu
        └── lidar

Each frame has ONE parent.
tf2 computes any transform by walking the tree.
```

| Direction | Operation |
|-----------|-----------|
| Parent → Child | Use transform as-is |
| Child → Parent | Reverse the transform |
```bash
# Publish static transform
ros2 run tf2_ros static_transform_publisher \
  --x 1.0 --y 0 --z 0.5 \
  --roll 0 --pitch 0 --yaw 0 \
  --frame-id world --child-frame-id drone

# Echo transform
ros2 run tf2_ros tf2_echo world drone
```

### Common Message Types

| Package | Message | Use |
|---------|---------|-----|
| `std_msgs` | `String`, `Int32`, `Float64` | Simple data |
| `sensor_msgs` | `Image` | Camera images |
| `sensor_msgs` | `PointCloud2` | 3D point clouds |
| `sensor_msgs` | `Imu` | Accelerometer/gyroscope |
| `geometry_msgs` | `Pose` | Position + orientation |
| `geometry_msgs` | `Transform` | Frame transform |
| `geometry_msgs` | `Twist` | Linear + angular velocity |

### Topics vs Services

| Topics | Services |
|--------|----------|
| Continuous stream | One-time request/response |
| Many subscribers | One client at a time |
| Publisher doesn't wait | Client waits for response |
| Sensor data, status | Commands, save/load, queries |
Each frame has ONE parent.
Chain transforms to go between frames.

---

## ROS2 Cheat Sheet

### Core Concepts
```
┌──────────┐     /topic      ┌──────────┐
│  Node A  │ ───────────────►│  Node B  │
│  (pub)   │    messages     │  (sub)   │
└──────────┘                 └──────────┘
```

| Concept | Description |
|---------|-------------|
| **Node** | Independent program that does one thing |
| **Topic** | Named channel for continuous data (like a radio frequency) |
| **Message** | Structured data sent on topics |
| **Publisher** | Sends data to a topic |
| **Subscriber** | Receives data from a topic |
| **Service** | Request/response (like a function call) |
| **Parameter** | Runtime configuration value |
| **Bag** | Recorded topic data for playback |
| **tf2** | Coordinate frame transform system |

### Command Line Tools
```bash
# Nodes
ros2 node list                    # List running nodes
ros2 node info /node_name         # Info about a node

# Topics
ros2 topic list                   # List all topics
ros2 topic echo /topic_name       # Listen to a topic
ros2 topic info /topic_name       # Show topic type and pub/sub count
ros2 topic hz /topic_name         # Show publish rate

# Messages
ros2 interface show std_msgs/msg/String    # Show message structure
ros2 interface show sensor_msgs/msg/Imu    # Show IMU message

# Services
ros2 service list                 # List all services
ros2 service call /srv_name type "{arg: value}"  # Call a service

# Parameters
ros2 param list /node_name        # List node's parameters
ros2 param get /node_name param   # Get parameter value
ros2 param set /node_name param value  # Set parameter

# Transforms (tf2)
ros2 run tf2_ros tf2_echo frame1 frame2   # Show transform between frames
ros2 run tf2_ros static_transform_publisher --x 1.0 --y 0 --z 0.5 --frame-id parent --child-frame-id child

# Bags
ros2 bag record /topic1 /topic2 -o bagname   # Record topics
ros2 bag play bagname                         # Playback recording
ros2 bag info bagname                         # Show bag info

# Running nodes
ros2 run package_name executable_name         # Run a node
ros2 launch package_name launch_file.py       # Run launch file
```

### Writing a Publisher Node
```cpp
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"

class MyPublisher : public rclcpp::Node {
public:
    MyPublisher() : Node("my_publisher") {
        // Create publisher on topic "my_topic" with queue size 10
        publisher_ = this->create_publisher<std_msgs::msg::String>("my_topic", 10);
        
        // Create timer that calls callback every 500ms
        timer_ = this->create_wall_timer(
            std::chrono::milliseconds(500),
            std::bind(&MyPublisher::timer_callback, this)
        );
    }

private:
    void timer_callback() {
        auto message = std_msgs::msg::String();
        message.data = "Hello World";
        publisher_->publish(message);
        RCLCPP_INFO(this->get_logger(), "Published: '%s'", message.data.c_str());
    }

    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr publisher_;
    rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char* argv[]) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<MyPublisher>());
    rclcpp::shutdown();
    return 0;
}
```

### Writing a Subscriber Node
```cpp
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"

class MySubscriber : public rclcpp::Node {
public:
    MySubscriber() : Node("my_subscriber") {
        subscription_ = this->create_subscription<std_msgs::msg::String>(
            "my_topic", 10,
            std::bind(&MySubscriber::topic_callback, this, std::placeholders::_1)
        );
    }

private:
    void topic_callback(const std_msgs::msg::String::SharedPtr msg) {
        RCLCPP_INFO(this->get_logger(), "Heard: '%s'", msg->data.c_str());
    }

    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr subscription_;
};

int main(int argc, char* argv[]) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<MySubscriber>());
    rclcpp::shutdown();
    return 0;
}
```

### Parameters
```cpp
// In constructor
this->declare_parameter("my_param", 100);           // Declare with default
int value = this->get_parameter("my_param").as_int();  // Get value

// Run with custom parameter
// ros2 run my_pkg my_node --ros-args -p my_param:=200
```

### CMakeLists.txt for ROS2
```cmake
cmake_minimum_required(VERSION 3.8)
project(my_package)

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)

# Create executable
add_executable(my_node src/my_node.cpp)
ament_target_dependencies(my_node rclcpp std_msgs)

# Install executable
install(TARGETS my_node
  DESTINATION lib/${PROJECT_NAME}
)

# Install launch files
install(DIRECTORY launch/
  DESTINATION share/${PROJECT_NAME}/launch
)

ament_package()
```

### package.xml
```xml
<?xml version="1.0"?>
<package format="3">
  <name>my_package</name>
  <version>0.0.1</version>
  <description>My ROS2 package</description>
  <maintainer email="you@email.com">Your Name</maintainer>
  <license>MIT</license>

  <buildtool_depend>ament_cmake</buildtool_depend>
  <depend>rclcpp</depend>
  <depend>std_msgs</depend>

  <export>
    <build_type>ament_cmake</build_type>
  </export>
</package>
```

### Launch File (Python)
```python
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='my_package',
            executable='my_publisher',
            name='publisher',
            parameters=[{'my_param': 100}]
        ),
        Node(
            package='my_package',
            executable='my_subscriber',
            name='subscriber'
        )
    ])
```

### Build and Run
```bash
# Create workspace
mkdir -p ~/ros2_ws/src
cd ~/ros2_ws/src

# Create package
ros2 pkg create --build-type ament_cmake my_package

# Build
cd ~/ros2_ws
colcon build

# Source (do this in every new terminal)
source install/setup.bash

# Run node
ros2 run my_package my_node

# Run launch file
ros2 launch my_package my_launch.py
```

### tf2 Transforms
```
TF Tree Example:
world
  └── drone
        ├── camera
        ├── imu
        └── lidar

Each frame has ONE parent.
tf2 computes any transform by walking the tree.
```

| Direction | Operation |
|-----------|-----------|
| Parent → Child | Use transform as-is |
| Child → Parent | Reverse the transform |
```bash
# Publish static transform
ros2 run tf2_ros static_transform_publisher \
  --x 1.0 --y 0 --z 0.5 \
  --roll 0 --pitch 0 --yaw 0 \
  --frame-id world --child-frame-id drone

# Echo transform
ros2 run tf2_ros tf2_echo world drone
```

### Common Message Types

| Package | Message | Use |
|---------|---------|-----|
| `std_msgs` | `String`, `Int32`, `Float64` | Simple data |
| `sensor_msgs` | `Image` | Camera images |
| `sensor_msgs` | `PointCloud2` | 3D point clouds |
| `sensor_msgs` | `Imu` | Accelerometer/gyroscope |
| `geometry_msgs` | `Pose` | Position + orientation |
| `geometry_msgs` | `Transform` | Frame transform |
| `geometry_msgs` | `Twist` | Linear + angular velocity |

### Topics vs Services

| Topics | Services |
|--------|----------|
| Continuous stream | One-time request/response |
| Many subscribers | One client at a time |
| Publisher doesn't wait | Client waits for response |
| Sensor data, status | Commands, save/load, queries |
'''