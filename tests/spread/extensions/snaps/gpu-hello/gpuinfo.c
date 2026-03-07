#include <EGL/egl.h>
#include <EGL/eglext.h>
#include <GL/gl.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#ifndef EGL_PLATFORM_SURFACELESS_MESA
#define EGL_PLATFORM_SURFACELESS_MESA 0x31DD
#endif

typedef EGLDisplay (*PFNEGLGETPLATFORMDISPLAYEXTPROC_LOCAL)(
    EGLenum platform,
    void *native_display,
    const EGLAttrib *attrib_list);

static const char *safe_string(const char *value)
{
    return value ? value : "(null)";
}

static void print_current_context_info(const char *profile_name)
{
    printf("%s vendor: %s\n", profile_name, safe_string((const char *)glGetString(GL_VENDOR)));
    printf("%s renderer: %s\n", profile_name, safe_string((const char *)glGetString(GL_RENDERER)));
    printf("%s version: %s\n", profile_name, safe_string((const char *)glGetString(GL_VERSION)));
    printf(
        "%s shading language version: %s\n",
        profile_name,
        safe_string((const char *)glGetString(GL_SHADING_LANGUAGE_VERSION)));
    printf("%s extensions:\n%s\n", profile_name, safe_string((const char *)glGetString(GL_EXTENSIONS)));
}

static EGLConfig choose_config(EGLDisplay display)
{
    EGLint attribs[] = {
        EGL_SURFACE_TYPE,
        EGL_PBUFFER_BIT,
        EGL_RENDERABLE_TYPE,
        EGL_OPENGL_BIT | EGL_OPENGL_ES2_BIT,
        EGL_NONE,
    };
    EGLConfig config = NULL;
    EGLint num_configs = 0;

    if (!eglChooseConfig(display, attribs, &config, 1, &num_configs) || num_configs < 1) {
        return NULL;
    }

    return config;
}

static void print_profile(
    EGLDisplay display,
    EGLConfig config,
    EGLenum api,
    const EGLint *context_attribs,
    EGLSurface surface,
    const char *profile_name)
{
    if (!eglBindAPI(api)) {
        fprintf(stderr, "failed to bind API for %s\n", profile_name);
        exit(EXIT_FAILURE);
    }

    EGLContext context = eglCreateContext(display, config, EGL_NO_CONTEXT, context_attribs);
    if (context == EGL_NO_CONTEXT) {
        fprintf(stderr, "failed to create context for %s\n", profile_name);
        exit(EXIT_FAILURE);
    }

    if (!eglMakeCurrent(display, surface, surface, context)) {
        eglDestroyContext(display, context);
        fprintf(stderr, "failed to make %s current\n", profile_name);
        exit(EXIT_FAILURE);
    }

    print_current_context_info(profile_name);

    eglMakeCurrent(display, EGL_NO_SURFACE, EGL_NO_SURFACE, EGL_NO_CONTEXT);
    eglDestroyContext(display, context);
}

int main(void)
{    const char *client_extensions = (const char *)eglQueryString(EGL_NO_DISPLAY, EGL_EXTENSIONS);
    if (!client_extensions) {
        client_extensions = "";
    }

    PFNEGLGETPLATFORMDISPLAYEXTPROC_LOCAL get_platform_display =
        (PFNEGLGETPLATFORMDISPLAYEXTPROC_LOCAL)eglGetProcAddress("eglGetPlatformDisplayEXT");

    if (!get_platform_display ||
        !strstr(client_extensions, "EGL_EXT_platform_base") ||
        !strstr(client_extensions, "EGL_MESA_platform_surfaceless")) {
        fprintf(stderr, "surfaceless platform is not supported\n");
        return EXIT_FAILURE;
    }

    EGLDisplay display = get_platform_display(EGL_PLATFORM_SURFACELESS_MESA, EGL_DEFAULT_DISPLAY, NULL);

    if (display == EGL_NO_DISPLAY) {
        fprintf(stderr, "failed to get surfaceless EGL display\n");
        return EXIT_FAILURE;
    }

    EGLint major = 0;
    EGLint minor = 0;
    if (!eglInitialize(display, &major, &minor)) {
        fprintf(stderr, "eglInitialize failed\n");
        return EXIT_FAILURE;
    }

    printf("EGL API version: %d.%d\n", major, minor);
    printf("EGL vendor string: %s\n", safe_string((const char *)eglQueryString(display, EGL_VENDOR)));
    printf("EGL version string: %s\n", safe_string((const char *)eglQueryString(display, EGL_VERSION)));
    printf("EGL client APIs: %s\n", safe_string((const char *)eglQueryString(display, EGL_CLIENT_APIS)));
    printf("EGL display extensions:\n%s\n", safe_string((const char *)eglQueryString(display, EGL_EXTENSIONS)));

    EGLConfig config = choose_config(display);
    if (!config) {
        fprintf(stderr, "failed to choose EGL config\n");
        eglTerminate(display);
        return EXIT_FAILURE;
    }

    const char *display_extensions = (const char *)eglQueryString(display, EGL_EXTENSIONS);
    EGLSurface surface = EGL_NO_SURFACE;
    EGLSurface cleanup_surface = EGL_NO_SURFACE;

    if (!display_extensions || !strstr(display_extensions, "EGL_KHR_surfaceless_context")) {
        EGLint pbuffer_attribs[] = {
            EGL_WIDTH,
            1,
            EGL_HEIGHT,
            1,
            EGL_NONE,
        };
        cleanup_surface = eglCreatePbufferSurface(display, config, pbuffer_attribs);
        if (cleanup_surface == EGL_NO_SURFACE) {
            fprintf(stderr, "failed to create fallback pbuffer surface\n");
            eglTerminate(display);
            return EXIT_FAILURE;
        }
        surface = cleanup_surface;
    }

    EGLint gl_core_attribs[] = {
        EGL_CONTEXT_MAJOR_VERSION,
        3,
        EGL_CONTEXT_MINOR_VERSION,
        3,
        EGL_CONTEXT_OPENGL_PROFILE_MASK,
        EGL_CONTEXT_OPENGL_CORE_PROFILE_BIT,
        EGL_NONE,
    };

    EGLint gl_compat_attribs[] = {
        EGL_CONTEXT_MAJOR_VERSION,
        3,
        EGL_CONTEXT_MINOR_VERSION,
        0,
        EGL_CONTEXT_OPENGL_PROFILE_MASK,
        EGL_CONTEXT_OPENGL_COMPATIBILITY_PROFILE_BIT,
        EGL_NONE,
    };

    EGLint gles_attribs[] = {
        EGL_CONTEXT_CLIENT_VERSION,
        2,
        EGL_NONE,
    };

    print_profile(display, config, EGL_OPENGL_API, gl_core_attribs, surface, "OpenGL core profile");
    print_profile(
        display,
        config,
        EGL_OPENGL_API,
        gl_compat_attribs,
        surface,
        "OpenGL compatibility profile");
    print_profile(display, config, EGL_OPENGL_ES_API, gles_attribs, surface, "OpenGL ES profile");

    EGLint num_configs = 0;
    if (eglGetConfigs(display, NULL, 0, &num_configs)) {
        printf("EGL configs: %d\n", num_configs);
    }

    if (cleanup_surface != EGL_NO_SURFACE) {
        eglDestroySurface(display, cleanup_surface);
    }

    eglTerminate(display);
    return EXIT_SUCCESS;
}
