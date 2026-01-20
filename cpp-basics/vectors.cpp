#include <iostream>
#include <vector>

int main() {
    std::vector<int> nums;  // Empty vector of ints

    nums.push_back(10);
    nums.push_back(20);
    nums.push_back(30);

    std::cout << "Size: " << nums.size() << std::endl;

    nums.pop_back();           // Remove last element
    std::cout << "After pop_back, size: " << nums.size() << std::endl;
    
    nums[0] = 99;              // Modify first element
    std::cout << "First element: " << nums.front() << std::endl;
    std::cout << "Last element: " << nums.back() << std::endl;
    
    for (int i = 0; i < nums.size(); i++) {
        std::cout << "nums[" << i << "] = " << nums[i] << std::endl;
    }

    std::cout << "\nRange-based loop:" << std::endl;
    for (int num : nums) {
        std::cout << num << std::endl;
    }

    return 0;
}