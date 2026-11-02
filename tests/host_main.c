int lte_run_tests(void);

int main(void)
{
    /* 0 = all tests passed; non-zero makes the CI job fail. */
    return (lte_run_tests() == 0) ? 0 : 1;
}