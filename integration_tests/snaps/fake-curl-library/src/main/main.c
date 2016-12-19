#include <stdio.h>
#include <curl/curl.h>

int main()
{
	printf("Starting\n");

	CURL *curl = curl_easy_init();
	if (curl)
	{
		curl_easy_cleanup(curl);
	}

	printf("Done\n");

	return 0;
}
