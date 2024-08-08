#include <stdio.h>
#include <stdlib.h>
#include <string.h>

int main(int argc, char const *argv[]) {
    const char *envs[] = {
        "GLOBAL",       // This is set globally
        "HELLO",        // This is set for the app
        "HELLO_WORLD",  // This is set from global env file
    };

    const char *expected[] = {
        "World",
        "Hello",
        "Hello World",
    };

    int num_envs = sizeof(envs) / sizeof(envs[0]);

    // Check Global, App specific and global env file
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
    const char *env_specific = getenv("SPECIFIC");
    if (env_specific != NULL) {
        fprintf(stderr, "\n[ERROR] Env. variable SPECIFIC is accessible from the app.\n");
        fprintf(stderr, "Expected: NULL\n");
        fprintf(stderr, "Got %s\n", env_specific);
        exit(1);
    }

    // Check if key with dot was rejected 
    const char *env_dot = getenv("DOT");
    if (env_dot != NULL) {
        fprintf(stderr, "\n[ERROR] Received Env. variable DOT with wrong key subset.\n");
        fprintf(stderr, "Expected: NULL\n");
        fprintf(stderr, "Got %s\n", env_dot);
        exit(1);
    }

    // Precedence check: Testing if the order of the envs is correct
    const char *env_order = getenv("ORDER");

    if (env_order == NULL){
        fprintf(stderr, "\n[ERROR] Env. variable ORDER is not set.\n");
        exit(1);
    }
    if (strcmp(env_order, "from app-specific") != 0) {
        fprintf(stderr, "\n[ERROR] Precedence error: app-specific envs should override global envs.\n");
        exit(1);
    }

    // Scope specific envfile
    const char *env_scope = getenv("SCOPED");

    if (env_scope == NULL){
        fprintf(stderr, "\n[ERROR] Env. variable SCOPED is not set.\n");
        exit(-1);
    }
    if (strcmp(env_scope, "Scoped") != 0) {
        fprintf(stderr, "\n[ERROR] app envfile error: unexpected value.\n");
        exit(-1);
    }

    return 0;
}
