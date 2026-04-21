/**
 * @file can_test.c
 * @brief Test framework implementation
 */

#include "can_test.h"
#include <string.h>

void test_suite_init(test_suite_t *suite)
{
    memset(suite, 0, sizeof(*suite));
}

void test_begin(test_suite_t *suite, const char *name)
{
    if (suite->total >= TEST_MAX_CASES) return;
    suite->cases[suite->total].name   = name;
    suite->cases[suite->total].result = TEST_PASS;  /* optimistic */
    suite->cases[suite->total].line   = 0;
}

void test_pass(test_suite_t *suite)
{
    if (suite->total >= TEST_MAX_CASES) return;
    suite->cases[suite->total].result = TEST_PASS;
    suite->passed++;
    suite->total++;
}

void test_fail(test_suite_t *suite, uint16_t line)
{
    if (suite->total >= TEST_MAX_CASES) return;
    suite->cases[suite->total].result = TEST_FAIL;
    suite->cases[suite->total].line   = line;
    suite->failed++;
    suite->total++;
}

void test_skip(test_suite_t *suite)
{
    if (suite->total >= TEST_MAX_CASES) return;
    suite->cases[suite->total].result = TEST_SKIP;
    suite->skipped++;
    suite->total++;
}
