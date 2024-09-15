#include "tst_lidarfiltering.h"
#include "tst_lazyevaluator.h"

int main(int argc, char **argv)
{
    int status = 0;

    //-- run all tests
    {
        TestLidarFiltering tc;
        status |= QTest::qExec(&tc, argc, argv);
    }

    {
        TestLazyEvaluator tc;
        status |= QTest::qExec(&tc, argc, argv);
    }

    return status;
}
