int lte_run_tests(void);
/* ============================================================================
   Test harness
   ============================================================================ */  

/**
 * Main function for the test harness.  
 * This function runs all the tests and returns 0 if all tests passed, or 1 if any test failed.     
 * @return 0 if all tests passed, 1 if any test failed.     
 *  
 */

int main(void)
{
    /* 0 = all tests passed; non-zero makes the CI job fail. */
    return (lte_run_tests() == 0) ? 0 : 1;
}