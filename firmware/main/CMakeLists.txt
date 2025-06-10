file(GLOB SRC_FILES "../src/*.c"
     GLOB TASK_FILES "../tasks/*.c"
     GLOB UTILS_FILES "../utils/*.c")  # Include all .c files in /src

idf_component_register(
    SRCS "init.c" "main.c" ${SRC_FILES} ${TASK_FILES} ${UTILS_FILES}   # Add main.c and all files from src
    INCLUDE_DIRS "." "../include" "../utils" # Add the external include directory
)
