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

set(PATH_TO_VERSION_TEMPLATE ${PROJECT_SOURCE_DIR}/cmake/version.hpp.in)
set(PATH_TO_VERSION_OUTPUT ${CMAKE_CURRENT_BINARY_DIR}/include/${PROJECT_NAME}/version.hpp)
if(NOT EXISTS ${PATH_TO_VERSION_TEMPLATE})
  message(FATAL_ERROR "Could not locate version template file at '${PATH_TO_VERSION_TEMPLATE}'.")
endif()

configure_file(
  ${PATH_TO_VERSION_TEMPLATE}
  ${PATH_TO_VERSION_OUTPUT}
  @ONLY
)
