#include <iostream>

extern "C" void hello() {
    std::cout << "Hello World!" << std::endl;
}
