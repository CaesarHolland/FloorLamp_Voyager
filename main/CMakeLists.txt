idf_component_register(SRCS "radar.c" "mymdns.c" "general_ctrl.c" "uart_ctrl.c" "main.c" "myhttp.c" "mywifi.c"
                    INCLUDE_DIRS "include" "html")
target_compile_options(${COMPONENT_LIB} PRIVATE "-Wno-format")