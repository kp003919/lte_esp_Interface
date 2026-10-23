idf_component_register(
    SRCS "lte_sm.c"
    INCLUDE_DIRS "include"
    REQUIRES lte_driver  comms lte_types
)