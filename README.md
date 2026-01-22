# Khronos Learning Journey

My path to understanding C++, ROS2, and the MIT SPARK stack for drone SLAM.

---

## 📊 Progress

| Phase | Topic | Status |
|-------|-------|--------|
| 1 | C++ Fundamentals | ✅ |
| 2 | CMake | ✅ |
| 3 | Math & Coordinate Frames | ✅ |
| 4 | ROS2 Fundamentals | ✅ |
| 5 | D455 Integration | ⏳ Waiting for hardware |
| 6 | GTSAM & Factor Graphs | ✅ |
| 7 | Kimera-VIO | ✅ |
| 8 | Kimera-Semantics | ✅ |
| 9 | Hydra | ✅ |
| 10 | Khronos | 🔜 |

**Completed:** ~23 hours | **Remaining:** ~2 hours

---

## 🗺️ The Big Picture: MIT SPARK Stack

```
D455 Camera (RGB + Depth + IMU)
         │
         ▼
┌─────────────────┐
│   Kimera-VIO    │ ──► "Where am I?" (poses)
└────────┬────────┘
         │
         ▼
┌─────────────────┐
│ Kimera-Semantics│ ──► "What's around me?" (labeled 3D mesh)
└────────┬────────┘
         │
         ▼
┌─────────────────┐
│     Hydra       │ ──► "How is it organized?" (scene graph)
└────────┬────────┘
         │
         ▼
┌─────────────────┐
│    Khronos      │ ──► "What changed?" (spatio-temporal)
└─────────────────┘
```

---

## 📚 External Resources

### Videos
- [MATLAB: Understanding SLAM Using Pose Graph Optimization](https://www.mathworks.com/videos/autonomous-navigation-part-3-understanding-slam-using-pose-graph-optimization-1594984678407.html) — Best visual explanation of factor graphs
- [Cyrill Stachniss: SLAM Course](https://www.youtube.com/playlist?list=PLgnQpQtFTOGQrZ4O5QzbIHgl3b1JHimN_) — Full university course on SLAM

### Documentation
- [ROS2 Humble Docs](https://docs.ros.org/en/humble/)
- [GTSAM Documentation](https://gtsam.org/docs/)
- [MIT SPARK Lab](https://web.mit.edu/sparklab/)

### Papers
- [Kimera: An Open-Source Library for Real-Time Metric-Semantic SLAM](https://arxiv.org/abs/1910.02490)
- [Hydra: A Real-time Spatial Perception System](https://arxiv.org/abs/2201.13360)
- [Khronos: A Unified Approach for Spatio-Temporal Metric-Semantic SLAM](https://arxiv.org/abs/2402.13817)

### GitHub Repos
- [Kimera-VIO](https://github.com/MIT-SPARK/Kimera-VIO)
- [Kimera-Semantics](https://github.com/MIT-SPARK/Kimera-Semantics)
- [Hydra](https://github.com/MIT-SPARK/Hydra)
- [Khronos](https://github.com/MIT-SPARK/Khronos)

---

# Phase 1: C++ Fundamentals

## Summary
C++ is the language of robotics. Unlike Python, you must declare types, manage memory, and think about performance.

## Key Concepts

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

# Phase 2: CMake

## Summary
CMake is the build system for C++. It figures out how to compile your code, link libraries, and manage dependencies.

## Key Concepts

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

# Phase 3: Math & Coordinate Frames

## Summary
Robotics requires understanding 3D transformations — how to represent position, orientation, and how to convert between different reference frames.

## Key Concepts

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
┌─────────────────┐
│ R  R  R │ tx    │
│ R  R  R │ ty    │
│ R  R  R │ tz    │
│ 0  0  0 │ 1     │
└─────────────────┘

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

---

# Phase 4: ROS2 Fundamentals

## Summary
ROS2 (Robot Operating System 2) is the middleware that connects everything. Nodes communicate via topics, services, and transforms.

## Core Concepts
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

## Command Line Tools
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

# Bags
ros2 bag record /topic1 /topic2 -o bagname   # Record topics
ros2 bag play bagname                         # Playback recording
ros2 bag info bagname                         # Show bag info

# Running nodes
ros2 run package_name executable_name         # Run a node
ros2 launch package_name launch_file.py       # Run launch file
```

## Writing a Publisher Node
```cpp
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"

class MyPublisher : public rclcpp::Node {
public:
    MyPublisher() : Node("my_publisher") {
        publisher_ = this->create_publisher<std_msgs::msg::String>("my_topic", 10);
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

## Writing a Subscriber Node
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

## CMakeLists.txt for ROS2
```cmake
cmake_minimum_required(VERSION 3.8)
project(my_package)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)

add_executable(my_node src/my_node.cpp)
ament_target_dependencies(my_node rclcpp std_msgs)

install(TARGETS my_node DESTINATION lib/${PROJECT_NAME})
install(DIRECTORY launch/ DESTINATION share/${PROJECT_NAME}/launch)

ament_package()
```

## package.xml
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

## Launch File (Python)
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

## Build and Run
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

# Run
ros2 run my_package my_node
ros2 launch my_package my_launch.py
```

## Common Message Types

| Package | Message | Use |
|---------|---------|-----|
| `std_msgs` | `String`, `Int32`, `Float64` | Simple data |
| `sensor_msgs` | `Image` | Camera images |
| `sensor_msgs` | `PointCloud2` | 3D point clouds |
| `sensor_msgs` | `Imu` | Accelerometer/gyroscope |
| `geometry_msgs` | `Pose` | Position + orientation |
| `geometry_msgs` | `Transform` | Frame transform |
| `geometry_msgs` | `Twist` | Linear + angular velocity |

## Topics vs Services

| Topics | Services |
|--------|----------|
| Continuous stream | One-time request/response |
| Many subscribers | One client at a time |
| Publisher doesn't wait | Client waits for response |
| Sensor data, status | Commands, save/load, queries |

---

# Phase 6: GTSAM & Factor Graphs

## Summary
Factor graphs are how SLAM systems represent and solve the localization problem. GTSAM (Georgia Tech Smoothing and Mapping) is the C++ library that optimizes them.

## The Rubber Band Analogy 🎯

Imagine you're blindfolded in a room, trying to figure out:
1. Where you walked (your path)
2. Where the furniture is (the map)

You have noisy measurements — your step counter says "1 meter" but maybe it was 0.95 or 1.05.

### The Model

**Variables = Balls that can move**
- Each position you stood at = a ball
- Each piece of furniture = a ball

**Factors = Rubber bands connecting balls**
- Your step counter = a rubber band between two consecutive position balls
- Bumping into furniture = a rubber band between your position ball and a furniture ball

```
    You at       You at       You at
    time 1       time 2       time 3
      🔵───────────🔵───────────🔵
       \           |
        \          |  (rubber bands)
         \         |
          🔴       🔴
        Chair     Table
```

Each rubber band has a **preferred length** (your measurement). The system **settles** into a configuration where all rubber bands are as close to their preferred length as possible.

### Loop Closure: The Magic

You walk in a circle and recognize "I'm back where I started!"

Now there's a rubber band connecting your first position to your last position — and it wants to be 0 meters long (same place).

```
    🔵───────────🔵───────────🔵
    ↑                          |
    |                          |
    └────── LOOP CLOSURE ──────┘
         (rubber band wants 
          length = 0)
```

This rubber band **pulls the whole chain** to correct accumulated drift. Loop closure is powerful because:
1. **High confidence** — recognizing "same place" is usually very certain
2. **Long reach** — it connects poses far apart in time, so correction ripples through entire trajectory

### Stiffness = Confidence

Not all measurements are equal:

| Measurement | Confidence | Rubber Band |
|-------------|------------|-------------|
| GPS "you're at [5, 10]" | High | Very stiff — hard to stretch |
| IMU "you moved 1m" | Medium | Normal stiffness |
| Wheel encoder (slippery floor) | Low | Loose — can stretch a lot |

When the system settles, **stiff rubber bands win**.

## Formal Vocabulary

| Rubber Band Analogy | Factor Graph Term |
|---------------------|-------------------|
| Balls (can move) | **Variables** — unknowns we estimate |
| Rubber bands | **Factors** — constraints from measurements |
| Rubber band's preferred length | **Measurement value** |
| Rubber band's stiffness | **Confidence/covariance** |
| System settling | **Optimization** |
| Loop closure rubber band | **Loop closure factor** |

## Types of Factors

| Factor Type | Connects | What it says |
|-------------|----------|--------------|
| **Odometry/IMU** | Pose → Pose | "I moved this much between these poses" |
| **Observation** | Pose → Landmark | "From this pose, I saw this landmark at this distance/angle" |
| **Prior** | Single variable | "I know this pose is roughly here" (like GPS) |
| **Loop closure** | Pose → Pose (distant) | "These two poses are the same place" |

## Visual
```
                    L1 (landmark)
                     ○
                    /|
     observation → / |
                  /  |
    prior → ■───○────■────○────■────○
           x1  odom  x2  odom  x3  odom  x4
                           \              /
                            \────────────/
                              loop closure
                     "x1 and x4 are the same place!"
```

## What GTSAM Does

GTSAM solves factor graphs by asking:

> "Where should I place all the balls so that the total stretch of all rubber bands is minimized?"

This is a nonlinear least squares optimization problem. GTSAM uses algorithms like Gauss-Newton and Levenberg-Marquardt to solve it efficiently.

## Key Insight

The optimizer finds the **best explanation** for all your noisy, conflicting measurements simultaneously. One bad measurement gets outvoted by many good ones.

---

# Phase 7: Kimera-VIO

## Summary
Kimera-VIO (Visual-Inertial Odometry) answers: **"Where am I and how am I moving?"** by fusing camera images and IMU data.

## Why Combine Camera + IMU?

| Sensor | Strengths | Weaknesses |
|--------|-----------|------------|
| Camera | Accurate position, rich features | Slow (30 fps), fails in blur/darkness |
| IMU | Fast (200+ Hz), works in any lighting | Drifts quickly, no absolute position |

Together they cover each other's weaknesses:
- **Camera:** "I see that corner is 2m away"
- **IMU:** "Between frames, you rotated 0.5° and moved 0.02m"
- **Combined:** Accurate pose at high speed

### Analogy
- **Walking blindfolded (IMU only):** You can feel each step, but after 100 steps you're lost
- **Walking with eyes only (camera only):** You can see where you are, but if you blink (motion blur), you're confused
- **Both:** Best of both worlds

## Architecture

```
┌─────────────────────────────────────────────────────┐
│                    FRONTEND                          │
│  • Detect features in images                        │
│  • Track features across frames                     │
│  • Process IMU data                                 │
└──────────────────────┬──────────────────────────────┘
                       │
                       ▼
┌─────────────────────────────────────────────────────┐
│                    BACKEND                           │
│  • Factor graph optimization (GTSAM)                │
│  • Estimate poses, velocity, IMU bias              │
│  • Output: camera trajectory                        │
└─────────────────────────────────────────────────────┘
```

## Frontend: Feature Detection & Tracking

### Step 1: Find Features
A "feature" is a distinctive point — corners, edges, blobs:

```
Image:
┌────────────────────────┐
│      ●                 │   ● = corner of window
│          ●             │   ● = edge of table
│   ●              ●     │   ● = corner of door
│              ●         │
└────────────────────────┘
```

Common algorithms: ORB, FAST, Shi-Tomasi corners

### Step 2: Track Features
Between frames, find the same features:

```
Frame 1:                    Frame 2:
┌────────────────────┐      ┌────────────────────┐
│      ●₁            │      │   ●₁               │
│          ●₂        │  →   │       ●₂           │
│   ●₃          ●₄   │      │●₃            ●₄    │
└────────────────────┘      └────────────────────┘

Features moved left → Camera moved right
```

**Key insight:** A flat white wall has no distinctive points. Every pixel looks the same. You can't track "which part of the wall" you're looking at. Corners and edges are unique and trackable.

## Frontend: IMU Preintegration

The camera runs at 30 fps, but the IMU runs at 200-1000 Hz:

```
Camera Frame 1                              Camera Frame 2
     │                                            │
     ▼                                            ▼
     📷─────────────────────────────────────────📷
        ↑  ↑  ↑  ↑  ↑  ↑  ↑  ↑  ↑  ↑  ↑  ↑  ↑
       IMU measurements (200 Hz between frames)
```

**Problem:** Adding 20 factors per camera frame = too expensive

**Solution:** Preintegration — combine all IMU measurements into ONE factor

```
Instead of:  Pose1 ──IMU── ──IMU── ──IMU── ──IMU── Pose2

You get:     Pose1 ─────────[Combined IMU Factor]───────── Pose2
```

No information lost — mathematically equivalent, just more efficient.

## Backend: Factor Graph

The backend builds a factor graph with these variables and factors:

### Variables
| Variable | What it is |
|----------|------------|
| Pose at time t | Position + orientation |
| Velocity at time t | How fast and which direction |
| IMU bias | Accelerometer and gyroscope drift |

### Factors
| Factor | What it connects |
|--------|------------------|
| Preintegrated IMU | Pose(t) → Pose(t+1), velocity, bias |
| Visual (feature tracks) | Pose(t) → Landmark |
| Prior | Initial pose (starting point) |

### Visual
```
Pose₁ ────────── Pose₂ ────────── Pose₃ ────────── Pose₄
  │    IMU+Vis     │    IMU+Vis     │    IMU+Vis     │
  ▼                ▼                ▼                ▼
Vel₁             Vel₂             Vel₃             Vel₄
  │                │                │                │
  └───────────────────────┬────────────────────────┘
                          ▼
              [  IMU Bias (shared across all)  ]

  │                │                │
  ▼                ▼                ▼
  ●────────────────●────────────────●
  Landmark₁        Landmark₂       (seen from multiple poses)
```

**Why is IMU bias shared?** It drifts slowly. One bias variable that evolves gradually makes physical sense. The optimizer figures out "the gyroscope has been drifting by 0.01 rad/s this whole time."

## What Happens When Camera Is Blocked?

Kimera-VIO doesn't completely fail. The IMU continues to provide motion estimates (**dead reckoning**).

However, without visual features:
- No new landmarks or feature tracks
- IMU drift accumulates quickly
- After a few seconds, position estimate becomes unreliable

It **degrades gracefully** rather than crashing. Once camera sees features again, visual factors help correct the drift.

## Output

| Output | What it is | Use |
|--------|------------|-----|
| Pose | Position (x, y, z) + orientation (quaternion) | Where is the camera? |
| Velocity | Linear velocity (vx, vy, vz) | How fast am I moving? |
| IMU Bias | Bias estimates | Used internally for correction |

### ROS2 Topics
```
/kimera_vio/odometry    → Pose + velocity
/tf                     → Transform: world → camera
```

---

# Phase 8: Kimera-Semantics

## Summary
Kimera-Semantics answers: **"What's around me?"** by building a labeled 3D mesh of the environment.

## What It Does

1. **3D Mesh Reconstruction** — Build a 3D model of the room
2. **Semantic Labeling** — Label every part ("wall", "floor", "chair", "table")

## Input → Output

```
Input:
  • Depth images from D455
  • Poses from Kimera-VIO
  • RGB images (for semantic labels)

Output:
  • 3D mesh of the environment
  • Each mesh face labeled ("floor", "wall", "chair", etc.)
```

## Pipeline

### Step 1: Depth to 3D Points

Your D455 gives a depth image — every pixel has a distance value:

```
Depth Image:
┌─────────────────────┐
│ 1.2  1.2  1.3  2.0  │  (meters to each pixel)
│ 1.2  1.2  1.3  2.1  │
│ 1.1  1.1  1.2  2.0  │
│ 0.8  0.8  0.8  0.8  │  ← floor is closer
└─────────────────────┘
```

Using camera intrinsics + pose from VIO:
```
Depth pixel (u, v, depth) + Pose → 3D point (x, y, z) in world
```

### Step 2: Build a Mesh

A point cloud is just dots. A mesh connects them into surfaces (triangles):

```
Point Cloud:              Mesh:
    •   •   •                 /\  /\
  •   •   •   •            /    \/    \
    •   •   •             /____________\
```

Kimera-Semantics uses **voxel hashing**:
1. Divide space into small cubes (voxels)
2. Track which voxels are occupied
3. Generate mesh surfaces where occupied meets empty

### Step 3: Semantic Segmentation

A neural network labels every pixel in the RGB image:

```
RGB Image:                    Segmentation Output:
┌─────────────────────┐       ┌─────────────────────┐
│   ┌───┐             │       │   WALL   WALL       │
│   │   │   window    │   →   │   WINDOW  WALL      │
│   └───┘             │       │   WALL   WALL       │
│ ▄▄▄▄▄▄▄             │       │   TABLE             │
│   table             │       │   FLOOR  FLOOR      │
└─────────────────────┘       └─────────────────────┘
```

**Note:** Kimera-Semantics is **network-agnostic** — you provide the segmentation output. Common choices:

| Network | Speed | Accuracy |
|---------|-------|----------|
| DeepLabV3 | Medium | High |
| SegFormer | Fast | High |
| Mask2Former | Slow | Very high |

### Step 4: Project 2D Labels onto 3D Mesh

Combine 2D labels with the 3D mesh:

```
For each face in the 3D mesh:
    1. Project it back into the camera image
    2. Look up what label that pixel has
    3. Assign that label to the mesh face
```

```
3D Mesh Face                 2D Image
     ▲                    ┌─────────────┐
    /|\                   │    WALL     │
   / | \   ──project──►   │      ●      │ ← this pixel = "wall"
  /  |  \                 │   FLOOR     │
 ▼───┴───▼                └─────────────┘

Result: That mesh face is labeled "wall"
```

### Step 5: Label Fusion Across Frames

You see the same surface from multiple viewpoints:
- Frame 1: Looks like "wall" (90% confident)
- Frame 2: Looks like "door" (60% confident)
- Frame 3: Looks like "wall" (85% confident)

**Solution: Probabilistic fusion** — keep a probability distribution for each face:

```
Mesh Face #42:
  wall:  0.75
  door:  0.20
  floor: 0.05
        ────
        1.00
```

Each new observation updates probabilities. Over time, the correct label wins.

**Key insight:** Any single neural network prediction can be wrong (lighting, occlusion, motion blur). By fusing across many frames, you're voting. One bad prediction gets outvoted by many good ones.

## Output: Metric-Semantic Mesh

```
         ceiling (gray)
        ┌──────────────────┐
       /                    \
      /     wall (white)     \
     │    ┌────┐              │
     │    │door│ (brown)      │  wall
     │    └────┘              │
     │   ▄▄▄▄▄▄               │
     │   table (tan)          │
     └────────────────────────┘
          floor (blue)
```

- **Metric** — accurate 3D geometry (positions in meters)
- **Semantic** — every face has a label

### ROS2 Topics
```
/kimera_semantics/mesh       → 3D mesh with labels
/kimera_semantics/pointcloud → Labeled point cloud
```

---

# Phase 9: Hydra

## Summary
Hydra builds a **3D Scene Graph** — a hierarchical representation that organizes the environment into objects, places, rooms, and buildings.

## The Problem Hydra Solves

Kimera-Semantics gives you a labeled mesh — but it's just a soup of triangles:

```
Mesh output:
  - 50,000 faces labeled "floor"
  - 12,000 faces labeled "wall"  
  - 800 faces labeled "chair"
  - 600 faces labeled "chair"
  - 400 faces labeled "table"
```

Questions you can't easily answer:
- How many chairs are there?
- Which room is the table in?
- What objects are near the door?

**Hydra organizes this chaos into a structured graph.**

## What is a Scene Graph?

A hierarchy of the environment:

```
                    ┌─────────────┐
                    │  BUILDING   │
                    └──────┬──────┘
                           │
          ┌────────────────┼────────────────┐
          ▼                ▼                ▼
    ┌──────────┐     ┌──────────┐     ┌──────────┐
    │  ROOM 1  │     │  ROOM 2  │     │  ROOM 3  │
    │ (kitchen)│     │ (bedroom)│     │ (hallway)│
    └────┬─────┘     └────┬─────┘     └──────────┘
         │                │
    ┌────┴────┐      ┌────┴────┐
    ▼         ▼      ▼         ▼
 ┌─────┐  ┌─────┐ ┌─────┐  ┌─────┐
 │chair│  │table│ │ bed │  │chair│
 └─────┘  └─────┘ └─────┘  └─────┘
```

It's like a family tree, but for spaces and objects.

## Hydra's 5 Layers

```
Layer 5:  BUILDING
             │
Layer 4:  ROOMS
             │
Layer 3:  PLACES (free space you can navigate)
             │
Layer 2:  OBJECTS
             │
Layer 1:  MESH (raw geometry from Kimera-Semantics)
```

| Layer | Contains | Question it answers |
|-------|----------|---------------------|
| 1 - Mesh | Raw triangles | "What's the geometry?" |
| 2 - Objects | Individual things (chair, table) | "What objects exist?" |
| 3 - Places | Navigable free space | "Where can I move?" |
| 4 - Rooms | Bounded areas | "What rooms are there?" |
| 5 - Building | Top-level container | "What building is this?" |

## Layer 2: Objects

Hydra clusters mesh faces into discrete objects:

```
Before (mesh):                After (objects):
  ▪▪▪▪                           ┌─────────┐
  ▪▪▪▪   ← 800 "chair" faces     │ Chair_1 │
  ▪▪▪▪                           │ pos: [2,3,0] │
                                 └─────────┘
  ▪▪▪▪                           ┌─────────┐
  ▪▪▪▪   ← 600 "chair" faces     │ Chair_2 │
  ▪▪▪▪                           │ pos: [5,1,0] │
                                 └─────────┘
```

Now you have discrete objects with positions, not just labeled soup.

## Layer 3: Places

A graph of navigable free space — where the robot can move:

```
┌─────────────────────────────────┐
│ Room                            │
│                                 │
│   ○───○───○───○                 │
│   │   │   │   │    ○ = place node    
│   ○───○───○───○    ─ = connection
│   │   │           (you can move between)
│   ○───○                         │
│       │                         │
│   [door]                        │
└─────────────────────────────────┘
```

Useful for path planning — "how do I get from here to there?"

## Layer 4: Rooms

Hydra groups places into rooms by detecting boundaries (walls, doors):

```
┌─────────────────┬─────────────────┐
│     ROOM 1      │     ROOM 2      │
│    (kitchen)    │    (bedroom)    │
│                 │                 │
│   ○───○───○    door   ○───○───○   │
│   │   │   │     │     │   │   │   │
│   ○───○───○─────┼─────○───○───○   │
│                 │                 │
│  [table]        │  [bed] [chair]  │
└─────────────────┴─────────────────┘
```

How Hydra knows room boundaries:
- Walls create separation in the places layer
- Doors create narrow connections between room clusters
- Algorithm: Graph clustering on the places layer

## Layer 5: Building

The top level — contains all rooms:

```
         ┌──────────────┐
         │   BUILDING   │
         └───────┬──────┘
                 │
    ┌────────────┼────────────┐
    ▼            ▼            ▼
┌────────┐  ┌────────┐  ┌────────┐
│ Room 1 │  │ Room 2 │  │ Room 3 │
└────────┘  └────────┘  └────────┘
```

## Edges: The Connections

The scene graph isn't just nodes — it's the **edges** (relationships) that make it powerful:

| Edge Type | Connects | Meaning |
|-----------|----------|---------|
| Parent-child | Room → Object | "This chair is IN this room" |
| Sibling | Object ↔ Object | "Chair is NEAR table" |
| Inter-layer | Place → Room | "This navigable spot is IN this room" |

**Key insight:** Without edges, you just have a list of objects. With edges, you can answer "what's in the kitchen?" or "what's near the door?" by traversing the graph.

## Real-Time Construction

Hydra builds incrementally as you explore:

```
Time 0s:   Drone starts
           └── Building
                └── Room_1 (partial)
                     └── Chair_1

Time 5s:   Drone explores more
           └── Building
                └── Room_1
                     ├── Chair_1
                     ├── Table_1
                     └── 12 place nodes

Time 15s:  Drone finds a door, enters new room
           └── Building
                ├── Room_1 (kitchen)
                │    ├── Chair_1
                │    └── Table_1
                └── Room_2 (bedroom)  ← NEW
                     └── Bed_1
```

## Output: Dynamic Scene Graph (DSG)

"Dynamic" because it changes over time as the robot explores.

### ROS2 Topics
```
/hydra/dsg          → Full scene graph
/hydra/dsg_mesh     → Mesh layer
/hydra/places       → Places layer visualization
```

---

# Phase 10: Khronos (Coming Soon)

## Preview

Khronos adds the **temporal** dimension: **"What changed?"**

All the previous systems assume a static world. Khronos handles:
- Objects that move
- Things that appear or disappear
- Changes over time

```
┌─────────────────────────────────────────────┐
│              SPATIO-TEMPORAL SLAM           │
│                                             │
│  Time 1: Chair at [2, 3]                    │
│  Time 2: Chair at [2, 3]  ✓ same            │
│  Time 3: Chair at [4, 5]  ⚠ MOVED!         │
│  Time 4: Chair GONE       ⚠ REMOVED!       │
└─────────────────────────────────────────────┘
```

This is the final piece — understanding not just *what* the world looks like, but *how* it changes.

---

## 🎯 What's Next?

1. **D455 arrives** → Phase 5: Hardware integration
2. **Complete Phase 10** → Khronos spatio-temporal concepts
3. **Build & run the full stack** → Live mapping!

---

*Last updated: January 2026*
