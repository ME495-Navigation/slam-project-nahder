
cmake_minimum_required(VERSION 3.22)

project(turtlelib)

find_package(Doxygen)

# To build documentation pass -DBUILD_DOCS=ON when generating the build system
option(BUILD_DOCS "Build the documentation" OFF)

# build just because Doxygen is missing
if(${DOXYGEN_FOUND} AND ${BUILD_DOCS})
    # Turn the README.md into the homepage of the doxygen docs
    set(DOXYGEN_USE_MDFILE_AS_MAINPAGE README.md)
    # Tell Doxygen where to find the documentation
    doxygen_add_docs(doxygen include/ src/ README.md ALL)
    # The documentation will be in the build/html directory
    # The main page is build/html/index.html
endif()

include(CTest)
# CTest sets BUILD_TESTING to on. To disable tests add -DBUILD_TESTING=OFF when invoking cmake
if(BUILD_TESTING)
    find_package(Catch2 3 REQUIRED)

    add_executable(test_geometry2d tests/test_geometry2d.cpp)
    add_executable(test_se2 tests/test_se2.cpp)
    add_executable(test_svg tests/test_svg.cpp)
    add_executable(test_diff_drive tests/test_diff_drive.cpp)

    target_link_libraries(test_geometry2d turtlelib Catch2::Catch2WithMain)
    target_link_libraries(test_se2 turtlelib Catch2::Catch2WithMain)
    target_link_libraries(test_svg turtlelib Catch2::Catch2WithMain)
    target_link_libraries(test_diff_drive turtlelib Catch2::Catch2WithMain)

    add_test(NAME Test_Geometry2D COMMAND test_geometry2d)
    add_test(NAME Test_SE2 COMMAND test_se2)
    add_test(NAME Test_SVG COMMAND test_svg)
    add_test(NAME Test_Diff_Drive COMMAND test_diff_drive)

endif()

add_library(turtlelib 
src/ekf.cpp src/geometry2d.cpp src/se2.cpp src/svg.cpp src/diff_drive.cpp)

add_executable(frame_main src/frame_main.cpp)

target_include_directories(turtlelib
    PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include/>
    $<INSTALL_INTERFACE:include/>)

target_compile_options(turtlelib PUBLIC -Wall -Wextra -pedantic)

target_compile_features(turtlelib PUBLIC cxx_std_17)

install(DIRECTORY include/turtlelib DESTINATION include)

target_link_libraries(frame_main turtlelib)

install(TARGETS frame_main turtlelib EXPORT turtlelib-targets)

install(EXPORT turtlelib-targets
        FILE turtlelib-config.cmake
        NAMESPACE turtlelib::
        DESTINATION lib/cmake/${PROJECT_NAME})  

        