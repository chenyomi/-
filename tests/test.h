#ifndef YOOMI_TEST_H
#define YOOMI_TEST_H

#include <stdio.h>

extern int g_fails;

#define EXPECT(cond)                                                                 \
    do {                                                                             \
        if (!(cond)) {                                                               \
            fprintf(stderr, "FAIL %s:%d: %s\n", __FILE__, __LINE__, #cond);         \
            g_fails++;                                                               \
        }                                                                            \
    } while (0)

#endif
