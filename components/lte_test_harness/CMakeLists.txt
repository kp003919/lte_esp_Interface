idf_component_register(
    SRCS "lte_test_harness.c"
    INCLUDE_DIRS "include"
    REQUIRES lte_driver comms hardware lte_types state_machine 
)
