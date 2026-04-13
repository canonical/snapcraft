#include <stdio.h>
#include <curl/curl.h>

int main() {
   curl_global_init(CURL_GLOBAL_DEFAULT);
   curl_version_info_data * data = curl_version_info(CURLVERSION_NOW);
   printf("curl version: %s\n", data->version);
   return 0;
}
