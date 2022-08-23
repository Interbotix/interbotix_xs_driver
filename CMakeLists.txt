cmake_minimum_required(VERSION 3.14)
project(interbotix_xs_driver LANGUAGES CXX)

include(CMakeDependentOption)
include(GNUInstallDirs)
include(InstallRequiredSystemLibraries)
include(CMakePackageConfigHelpers)

add_subdirectory(third_party_libraries/DynamixelSDK)
add_subdirectory(third_party_libraries/dynamixel-workbench)

find_package(yaml-cpp REQUIRED)

set(DEPENDENCIES
  dynamixel_workbench_toolbox
  yaml-cpp
)

# Generate version header file from package.xml
include(cmake/Version.cmake)
message(${PATH_TO_VERSION_OUTPUT_DIR})
include_directories(${PATH_TO_VERSION_OUTPUT_DIR})

include_directories(${CMAKE_CURRENT_SOURCE_DIR}/include)

add_library(
  ${PROJECT_NAME} SHARED
    src/xs_driver.cpp
    src/xs_logging.cpp
)

target_compile_features(${PROJECT_NAME} PRIVATE cxx_std_17)

target_include_directories(${PROJECT_NAME} PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_BINARY_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

target_link_libraries(${PROJECT_NAME}
  ${DEPENDENCIES}
)

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
