/* gcc -Wall -O2 -o bindtextdomain.so -fPIC -shared bindtextdomain.c -ldl */

#define _GNU_SOURCE

#include <errno.h>
#include <dirent.h>
#include <dlfcn.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>

static void die (const char *msg)
{
        fprintf (stderr, "can't preload: %s\n", msg);
        exit (EXIT_FAILURE);
}

typedef char * (*BindTextDomainFunc) (const char *a,
                                      const char *b);

static BindTextDomainFunc r_bindtextdomain = NULL;

char *bindtextdomain(const char *domainname, const char *dirname)
{
        char *snap = NULL;
        char *snap_path = NULL;
        char *snap_locale_path = NULL;
        DIR *dir = NULL;
        struct dirent *ent = NULL;
        char *ret = NULL;

        if (r_bindtextdomain == 0 && !(r_bindtextdomain =
                                       (BindTextDomainFunc) dlsym (RTLD_NEXT, "bindtextdomain")))
                die ("can't find symbol \"bindtextdomain\"");

        if (dirname == NULL || dirname[0] != '/')
                goto orig;

        snap = getenv ("SNAP");

        if (snap == NULL || strcmp (snap, "") == 0)
                goto orig;

        char * paths[] = {
                "gnome-platform/usr/share/locale",
                "gnome-platform/usr/share/locale-langpack",
                "usr/share/locale",
                "usr/share/locale-langpack",
                NULL
        };
        for(int i = 0; paths[i] != NULL; i++)
        {
                if (asprintf (&snap_path, "%s/%s", snap, paths[i]) < 0)
                        continue;

                if (access (snap_path, F_OK) < 0) {
                        free(snap_path);
                        snap_path = NULL;
                        continue;
                }

                /*
                 * if the mo file exists for one language we assume it exists for them
                 * all, or at least that we're not going to find it anywhere else. we
                 * don't know at this point what locale the application is actually
                 * going to use, so we can't look in any particular directory.
                 */
                dir = opendir (snap_path);
                if (dir == NULL) {
                        free (snap_path);
                        snap_path = NULL;
                        continue;
                }

                while ((ent = readdir (dir)) != NULL) {
                        if (ent->d_name[0] == '.')
                                continue;

                        if (asprintf (&snap_locale_path,
                                      "%s/%s/LC_MESSAGES/%s.mo",
                                      snap_path,
                                      ent->d_name,
                                      domainname) < 0)
                                continue;

                        /* snap_locale_path has been allocated if we made it
                         * this far, be sure it's freed before any goto
                         * or continue
                         */
                        if (access (snap_locale_path, F_OK) == 0) {
                                closedir (dir);
                                free (snap_locale_path);
                                snap_locale_path = NULL;
                                goto ok;
                        } else {
                                free (snap_locale_path);
                                snap_locale_path = NULL;
                                continue;
                        }
                }

                closedir (dir);
                free (snap_path);
                snap_path = NULL;
        }
        /*
         * we fell out of the loop, so we'll go to orig regardless - no need to
         * check for errors
         */
        goto orig;

ok:
        ret = r_bindtextdomain (domainname, snap_path);
        free (snap_path);
        snap_path = NULL;
        goto out;
orig:
        ret = r_bindtextdomain (domainname, dirname);
        goto out;
out:
        return ret;
}
