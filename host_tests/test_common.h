/* Framework test mini (tanpa dependensi). */
#ifndef TEST_COMMON_H
#define TEST_COMMON_H
#include <stdio.h>
#include <math.h>
#include <stdlib.h>

static int g_checks = 0, g_fails = 0;

#define CHECK(cond) do { g_checks++; if (!(cond)) { g_fails++; \
    printf("  FAIL %s:%d: %s\n", __FILE__, __LINE__, #cond); } } while (0)

#define CHECK_NEAR(a, b, tol) do { g_checks++; double _a=(a), _b=(b); \
    if (!(fabs(_a-_b) <= (tol))) { g_fails++; \
    printf("  FAIL %s:%d: %s = %.6f, expected %.6f (+-%g)\n", __FILE__, __LINE__, #a, _a, _b, (double)(tol)); } } while (0)

#define TEST_SUMMARY(name) do { printf("%-22s checks=%d fails=%d  %s\n", name, g_checks, g_fails, \
    g_fails ? "FAILED" : "OK"); return g_fails ? 1 : 0; } while (0)

/* selisih sudut terpendek (untuk assert heading), independen dari firmware */
static inline double ang_diff(double a, double b) {
    double d = fmod(a - b, 360.0);
    if (d > 180.0) d -= 360.0;
    if (d < -180.0) d += 360.0;
    return d;
}
#endif
