#include <rtthread.h>
#include <finsh.h>

#include <g1632.h>

int g1632_test(int argc, char** argv)
{
    hello_func();

    return 0;
}
MSH_CMD_EXPORT(g1632_test, g1632 API test);
