#include <stdio.h>
#include <stdlib.h>
#include <string.h>

int main(int argc, char const *argv[]) {
    const char *envs[] = {
        "HELLO",        // This is set globally
        "WORLD",        // This is set for the app
        "HELLO_WORLD",  // This is set from env file
    };

    const char *expected[] = {
        "Hello",
        "World",
        "Hello World",
    };

    int num_envs = sizeof(envs) / sizeof(envs[0]);

    for (int i = 0; i < num_envs; ++i) {
        const char *env = getenv(envs[i]);

        if (env == NULL) {
            fprintf(stderr, "\n[ERROR] Env. variable %s is not set.\n", envs[i]);
            return 1;
        }

        if (strcmp(env, expected[i]) != 0) {
            fprintf(stderr, "\n[ERROR] Env. variable %s isn't set to the expected value.\n", envs[i]);
            fprintf(stderr, "Expected: %s\n", expected[i]);
            fprintf(stderr, "Got: %s\n", env);
            return 1;
        }
    }

    return 0;
}
