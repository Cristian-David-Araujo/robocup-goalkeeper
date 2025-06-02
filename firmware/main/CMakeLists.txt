file(GLOB SRC_FILES "../src/*.c"
     GLOB TASK_FILES "../tasks/*.c")  # Include all .c files in /src

idf_component_register(
    SRCS "main.c" ${SRC_FILES}     # Add main.c and all files from src
    INCLUDE_DIRS "." "../include"  # Add the external include directory
)
