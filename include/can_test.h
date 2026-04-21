/**
 * @file can_test.h
 * @brief Lightweight test framework for the LPC1768 CAN driver
 *
 * Provides assertion macros that log pass/fail to an in-RAM
 * results buffer.  After all tests run, main.c reads the
 * results and blinks LED codes.
 *
 * Week 4 — Testing & Deployment
 */

#ifndef CAN_TEST_H
#define CAN_TEST_H

#include <stdint.h>
#include <stdbool.h>

/* ── Result codes ──────────────────────────────────────────── */
typedef enum {
    TEST_PASS = 0,
    TEST_FAIL = 1,
    TEST_SKIP = 2
} test_result_t;

/* ── Single test-case record ───────────────────────────────── */
typedef struct {
    const char    *name;
    test_result_t  result;
    uint16_t       line;       /**< Source line of failure    */
} test_record_t;

/* ── Overall results summary ───────────────────────────────── */
#define TEST_MAX_CASES  48

typedef struct {
    test_record_t cases[TEST_MAX_CASES];
    uint8_t       total;
    uint8_t       passed;
    uint8_t       failed;
    uint8_t       skipped;
} test_suite_t;

/* ── API ───────────────────────────────────────────────────── */

/** Initialise / reset the test suite */
void test_suite_init(test_suite_t *suite);

/** Begin a named test case (call before assertions) */
void test_begin(test_suite_t *suite, const char *name);

/** Record a pass for the current test */
void test_pass(test_suite_t *suite);

/** Record a fail for the current test with source line */
void test_fail(test_suite_t *suite, uint16_t line);

/** Skip the current test (e.g. hardware not available) */
void test_skip(test_suite_t *suite);

/* ── Macros ────────────────────────────────────────────────── */

#define TEST_BEGIN(suite, name)  test_begin((suite), (name))

#define TEST_ASSERT(suite, cond)              \
    do {                                      \
        if (cond) { test_pass(suite); }       \
        else      { test_fail(suite, __LINE__); } \
    } while (0)

#define TEST_ASSERT_EQ(suite, a, b)           \
    TEST_ASSERT((suite), (a) == (b))

#define TEST_ASSERT_NEQ(suite, a, b)          \
    TEST_ASSERT((suite), (a) != (b))

#define TEST_SKIP(suite)  test_skip(suite)

#endif /* CAN_TEST_H */
