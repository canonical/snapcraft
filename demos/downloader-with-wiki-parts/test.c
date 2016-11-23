#include <stdio.h>
#include <curl/curl.h>

int main(void)
{
  CURL *curl;

  curl = curl_easy_init();
  if(curl) {
    curl_easy_setopt(curl, CURLOPT_URL, "http://start.ubuntu.com/connectivity-check.html");
    curl_easy_perform(curl);

    curl_easy_cleanup(curl);
  }

  return 0;
}
