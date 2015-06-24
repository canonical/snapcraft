#include <pipeline.h>

int main()
{
    pipeline *p;
    int status;

    p = pipeline_new ();
    pipeline_command_args (p, "ls", NULL);
    pipeline_command_args (p, "grep", "s", NULL);
    pipeline_command_args (p, "grep", "t", NULL);
    status = pipeline_run (p);

    return status;
}
