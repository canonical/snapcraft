#include <stdio.h>

#if HELLO == 1
static char *say = "hello world";
#else
static char *say = "goodbye world";
#endif

int main()
{
  printf("%s\n", say);
  return 0;
}
