file(GLOB SRC_FILES "../src/*.c")
file(GLOB TASK_FILES "../tasks/*.c")
file(GLOB UTILS_FILES "../utils/*.c")

idf_component_register(
    SRCS "init.c" "main.c" ${SRC_FILES} ${TASK_FILES} ${UTILS_FILES}
    INCLUDE_DIRS "." "../include" "../utils"
)
