# Set up the project.
cmake_minimum_required (VERSION 2.4)
project (qr)

# Add compiler flags.
add_definitions (-Wall -O2 -g)

# Make sure the compiler can find the include files.
include_directories (include)

# List the source files here.
set (SRCS src/qr)

# Put the executable in a common directory.
set (EXECUTABLE_OUTPUT_PATH bin)

# Build the executable.
add_executable (${PROJECT_NAME} ${SRCS})

# Link to the math library.
IF (UNIX)
  target_link_libraries (${PROJECT_NAME} m)
ENDIF (UNIX)

