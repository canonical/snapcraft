#include <stdio.h>
#include <curl/curl.h>
#include <png.h>

int main() {
    curl_global_init(CURL_GLOBAL_DEFAULT);
    png_create_read_struct(PNG_LIBPNG_VER_STRING, NULL, NULL, NULL);
}
