#include <stdio.h>
#include <stdlib.h>
#include <string.h>

int main(int argc, char const *argv[]) {
    const char *envs[] = {
        "GLOBAL",       // This is set globally
        "SPECIFIC",     // This is set for the app only
        "HELLO_WORLD",  // This is set from global env file
    };

    const char *expected[] = {
        "World",
        "City",
        "Hello World",
    };

    int num_envs = sizeof(envs) / sizeof(envs[0]);

    for (int i = 0; i < num_envs; ++i) {
        const char *env = getenv(envs[i]);

        if (env == NULL) {
            fprintf(stderr, "\n[ERROR] Env. variable %s is not set.\n", envs[i]);
            exit(1);
        }

        if (strcmp(env, expected[i]) != 0) {
            fprintf(stderr, "\n[ERROR] Env. variable %s isn't set to the expected value.\n", envs[i]);
            fprintf(stderr, "Expected: %s\n", expected[i]);
            fprintf(stderr, "Got: %s\n", env);
            exit(1);
        }
    }

    // Check that it's not possible to access other app ENVs 
    const char *env_hello = getenv("HELLO");
    if (env_hello != NULL) {
        fprintf(stderr, "\n[ERROR] Env. variable SPECIFIC is accessible from the app.\n");
        fprintf(stderr, "Expected: NULL\n");
        fprintf(stderr, "Got %s\n", env_hello);
        exit(1);
    }

    return 0;
}
