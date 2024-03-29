# Basic parameters; check that these match your project / environment
cmake_minimum_required(VERSION 3.9)

# Replace "game" with a name for your project (this is used the name of the output)
project(ti99)

# Add your sources here (adding headers is optional, but helps some CMake generators)
# add_compile_definitions(VERIFY)
# set(PROJECT_SOURCE ti99.cpp ti99.hpp vdptest.c tms9900.cpp 994arom.c 994agrom.c cpu9900.cpp tursicpu.cpp invadersrom.c invadersgrom.c)

# Normal execution without verify below:
set(PROJECT_SOURCE ti99.cpp ti99.hpp vdptest.c tms9900.cpp 
  tms9918.cpp sys.cpp
  994arom.c 994agrom.c invadersrom.c invadersgrom.c
  multicolorrom.c
  )



# ... and any other files you want in the release here
set(PROJECT_DISTRIBS LICENSE README.md)

# Build configuration; approach this with caution!
if(MSVC)
  add_compile_options("/W4" "/wd4244" "/wd4324" "/wd4458" "/wd4100")
else()
  add_compile_options("-Wall" "-Wextra" "-Wdouble-promotion" "-Wno-unused-parameter")
endif()

find_package (32BLIT CONFIG REQUIRED PATHS ../32blit-sdk $ENV{PATH_32BLIT_SDK})

blit_executable (${PROJECT_NAME} ${PROJECT_SOURCE})
blit_assets_yaml (${PROJECT_NAME} assets.yml)
blit_metadata (${PROJECT_NAME} metadata.yml)
add_custom_target (flash DEPENDS ${PROJECT_NAME}.flash)

# setup release packages
install (FILES ${PROJECT_DISTRIBS} DESTINATION .)
set (CPACK_INCLUDE_TOPLEVEL_DIRECTORY OFF)
set (CPACK_GENERATOR "ZIP" "TGZ")
include (CPack)
