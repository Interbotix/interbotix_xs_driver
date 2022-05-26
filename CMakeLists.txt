cmake_minimum_required(VERSION 3.5)
project(interbotix_xs_driver LANGUAGES CXX)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

if(CMAKE_COMPILER_IS_GNUXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(dynamixel_workbench_toolbox REQUIRED)
find_package(yaml-cpp REQUIRED)

include_directories(include)

set(DEPENDENCIES
  dynamixel_workbench_toolbox
)

# generate version header file from package.xml
file(READ "${PROJECT_SOURCE_DIR}/package.xml" PACKAGE_MANIFEST)
string(REGEX MATCH "<version>([0-9]).[0-9].[0-9]</version>" _ ${PACKAGE_MANIFEST})
set(${PROJECT_NAME}_VERSION_MAJOR ${CMAKE_MATCH_1})
string(REGEX MATCH "<version>[0-9].([0-9]).[0-9]</version>" _ ${PACKAGE_MANIFEST})
set(${PROJECT_NAME}_VERSION_MINOR ${CMAKE_MATCH_1})
string(REGEX MATCH "<version>[0-9].[0-9].([0-9])</version>" _ ${PACKAGE_MANIFEST})
set(${PROJECT_NAME}_VERSION_PATCH ${CMAKE_MATCH_1})
set(${PROJECT_NAME}_VERSION ${${PROJECT_NAME}_VERSION_MAJOR}.${${PROJECT_NAME}_VERSION_MINOR}.${${PROJECT_NAME}_VERSION_PATCH})
if(NOT ${PROJECT_NAME}_VERSION MATCHES "[0-9].[0-9].[0-9]")
  message(FATAL_ERROR "Could not parse version from package.xml or version is not formatted properly (should be MAJOR.MINOR.PATCH).")
endif()

set(PATH_TO_VERSION_TEMPLATE ${PROJECT_SOURCE_DIR}/include/${PROJECT_NAME}/version.hpp.in)
set(PATH_TO_VERSION_OUTPUT ${CMAKE_CURRENT_BINARY_DIR}/include/${PROJECT_NAME}/version.hpp)
if(NOT EXISTS ${PATH_TO_VERSION_TEMPLATE})
  message(FATAL_ERROR "Could not locate version template file at '${PATH_TO_VERSION_TEMPLATE}'.")
endif()

configure_file(
  ${PATH_TO_VERSION_TEMPLATE}
  ${PATH_TO_VERSION_OUTPUT}
  @ONLY
)

include_directories(${CMAKE_CURRENT_BINARY_DIR}/include)

add_library(
  ${PROJECT_NAME} SHARED
    src/xs_driver.cpp
    src/xs_logging.cpp
)

ament_target_dependencies(${PROJECT_NAME} ${DEPENDENCIES})
target_include_directories(${PROJECT_NAME} PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)
target_link_libraries(${PROJECT_NAME} yaml-cpp)

install(
  TARGETS ${PROJECT_NAME}
  EXPORT "export_${PROJECT_NAME}"
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION lib
  INCLUDES DESTINATION include
)

install(
  FILES
    ${PATH_TO_VERSION_OUTPUT}
  DESTINATION
    include/${PROJECT_NAME}
)

install(
  DIRECTORY
    include/${PROJECT_NAME}/
  DESTINATION
    include/${PROJECT_NAME}
)

install(
  FILES
    package.xml
  DESTINATION
    share/${PROJECT_NAME}
)

ament_export_dependencies(
  yaml-cpp
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_export_targets("export_${PROJECT_NAME}")
ament_package()
