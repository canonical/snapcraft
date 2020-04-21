#include <stdio.h>

#if HELLO == 1
static char *say = "hello";
#else
static char *say = "goodbye";
#endif

int main()
{
  printf("%s world\n", say);
  return 0;
}
