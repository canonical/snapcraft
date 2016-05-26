#include <pipeline.h>
#include <stdio.h>

int main()
{
    pipeline *p;
    int status;

    printf("running echo test | grep s | grep t\n");

    p = pipeline_new ();
    pipeline_command_args (p, "echo", "test", NULL);
    pipeline_command_args (p, "grep", "s", NULL);
    pipeline_command_args (p, "grep", "t", NULL);
    status = pipeline_run (p);

    return status;
}
