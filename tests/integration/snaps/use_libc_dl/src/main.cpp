#include <cstdlib>
#include <iostream>
#include <dlfcn.h>

int main() {
    // Determine library path
    std::string snap(std::getenv("SNAP"));
    if (snap.empty())
    {
        std::cerr << "The $SNAP environment variable is not defined" << std::endl;
        return 1;
    }

    std::string library_path(snap + "/lib/libhello.so");

    // Open the library
    std::cout << "Opening lib" << std::endl;
    void *handle = dlopen(library_path.c_str(), RTLD_LAZY);
    
    if (!handle)
    {
        std::cerr << "Cannot open lib: " << dlerror() << std::endl;
        return 1;
    }
    
    typedef void (*VoidFunctionPtr)();

    // Loading "hello" symbol
    std::cout << "Loading symbol 'hello'" << std::endl;
    VoidFunctionPtr hello = reinterpret_cast<VoidFunctionPtr>(dlsym(handle, "hello"));
    const char *error = dlerror();
    if (error)
    {
        std::cerr << "Cannot load symbol 'hello' from lib: " << error
                  << std::endl;
        dlclose(handle);
        return 1;
    }
    
    // Now actually call the hello function out of the dlopened library
    std::cout << "Calling hello" << std::endl;
    hello();
    
    // Finally, clean up
    std::cout << "Closing lib" << std::endl;
    dlclose(handle);

    return 0;
}
