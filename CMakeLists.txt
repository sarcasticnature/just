cmake_minimum_required(VERSION 3.22)
project(just)

set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wall -Wextra -pedantic -Wno-missing-field-initializers")

set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

if(NOT CMAKE_BUILD_TYPE)
  set(CMAKE_BUILD_TYPE "RelWithDeb")
endif()

option(BOX2D_BUILD_TESTBED OFF)
option(BOX2D_BUILD_UNIT_TESTS OFF)

include(FetchContent)

FetchContent_Declare(
	box2d
	GIT_REPOSITORY https://github.com/erincatto/box2d.git
	GIT_TAG main
	GIT_SHALLOW TRUE
	GIT_PROGRESS TRUE
)
FetchContent_MakeAvailable(box2d)

FetchContent_Declare(
	raylib
	GIT_REPOSITORY https://github.com/raysan5/raylib.git
	GIT_TAG 5.0
	GIT_SHALLOW TRUE
	GIT_PROGRESS TRUE
)
FetchContent_MakeAvailable(raylib)

FetchContent_Declare(
    tomlplusplus
    GIT_REPOSITORY https://github.com/marzer/tomlplusplus.git
	GIT_SHALLOW TRUE
    GIT_TAG v3.4.0
)
FetchContent_MakeAvailable(tomlplusplus)

include(FetchContent)
FetchContent_Declare(
    doctest
    GIT_REPOSITORY https://github.com/doctest/doctest
    GIT_TAG v2.4.11
	GIT_SHALLOW TRUE
)
FetchContent_MakeAvailable(doctest)

option(JUST_BUILD_TESTS "whether or not to build the tests" ON)

include_directories(include)

set(just_srcs
    src/world_model.cpp
)

set(just_deps
    box2d
    raylib
    tomlplusplus::tomlplusplus
    doctest::doctest
)

add_library(just ${just_srcs})
target_compile_definitions(just PUBLIC DOCTEST_CONFIG_DISABLE)
target_link_libraries(just PRIVATE ${just_deps})

add_executable(demo src/demo.cpp)
target_link_libraries(demo PRIVATE just ${just_deps})

add_executable(grid_demo src/grid_demo.cpp)
target_link_libraries(grid_demo PRIVATE just ${just_deps})

if(JUST_BUILD_TESTS)
    add_executable(tests test/doctest_main.cpp ${just_srcs})
    target_link_libraries(tests ${just_deps})
endif()
