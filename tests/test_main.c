#include "test.h"

#include <stdio.h>

int g_fails;

void test_motion(void);
void test_protocol(void);
void test_cell(void);

int main(void)
{
    test_motion();
    test_protocol();
    test_cell();
    if (g_fails != 0) {
        fprintf(stderr, "%d assertion(s) failed\n", g_fails);
        return 1;
    }
    printf("ok\n");
    return 0;
}
