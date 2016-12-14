#include <stdio.h>

void *curl_easy_init()
{
	printf("Fake init\n");
	return 1;
}

void curl_easy_cleanup(void *curl)
{
	printf("Fake cleanup\n");
}
