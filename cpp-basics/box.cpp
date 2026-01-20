#include <iostream>
#include <string>

template <typename T>
class Box {
private:
    T contents;
    
public:
    Box(T item) {
        contents = item;
    }
    
    T getContents() {
        return contents;
    }
    
    void setContents(T item) {
        contents = item;
    }
};

int main() {
    Box<int> intBox(42);
    Box<std::string> stringBox("Hello");
    
    std::cout << "Int box: " << intBox.getContents() << std::endl;
    std::cout << "String box: " << stringBox.getContents() << std::endl;
    
    intBox.setContents(100);
    std::cout << "Int box now: " << intBox.getContents() << std::endl;
    
    return 0;
}