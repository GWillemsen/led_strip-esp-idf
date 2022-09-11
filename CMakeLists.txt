set(SRCS "src/led_strip.c")

idf_component_register(
        SRCS ${SRCS}
        INCLUDE_DIRS ./include
)