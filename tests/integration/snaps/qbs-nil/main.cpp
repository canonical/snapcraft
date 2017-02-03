#include <stdio.h>

int main(int argc, char *argv[])
{
#ifdef PRINT_STRING
    printf("Hello Snapcraft World");
#else
    printf("Wrong snapcraft string");
#endif
    return 0;
}
