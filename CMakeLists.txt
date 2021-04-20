#################### BASIC SETUP ####################
cmake_minimum_required(VERSION 3.16)
project(InEKF VERSION 0.1 LANGUAGES CXX)

#Setup FetchContent
include(FetchContent)
if(${CMAKE_VERSION} VERSION_LESS 3.14)
  macro(FetchContent_MakeAvailable NAMES)
    foreach(NAME ${NAMES})
      FetchContent_GetProperties(${NAME})
      if(NOT ${NAME}_POPULATED)
        FetchContent_Populate(${NAME})
        add_subdirectory(${${NAME}_SOURCE_DIR} ${${NAME}_BINARY_DIR})
      endif()
    endforeach(NAME)
  endmacro() 
endif()

#################### IMPORT DEPENDENCIES ####################
set(EXAMPLES 1)

# Eigen 
find_package(Eigen3 CONFIG REQUIRED)

# Python for plotting and/or wrapper
if(PYTHON OR EXAMPLES)
  find_package(Python3 COMPONENTS Interpreter Development NumPy)
endif()

# Google Test
FetchContent_Declare(
    googletest
    GIT_REPOSITORY https://github.com/google/googletest.git
    GIT_TAG        release-1.10.0
)
FetchContent_GetProperties(googletest)
if(NOT googletest_POPULATED)
    FetchContent_Populate(googletest)
    add_subdirectory(${googletest_SOURCE_DIR} ${googletest_BINARY_DIR})
endif()


#################### ADD TARGETS ####################
add_subdirectory(src)
if(EXAMPLES)
  add_subdirectory(exe)
endif()


#################### ADD TESTS ####################
enable_testing()
add_subdirectory(tests)


#################### INSTALL AS LIBRARY ####################
# set(INSTALL_LIBS Core SE2_3_Bias)
# install(TARGETS ${INSTALL_LIBS} EXPORT ${PROJECT_NAME}Config
#         ARCHIVE DESTINATION lib
#         LIBRARY DESTINATION lib
#         RUNTIME DESTINATION bin)

# install(DIRECTORY "${CMAKE_SOURCE_DIR}/include" # source directory
#         DESTINATION "${CMAKE_INSTALL_PREFIX}" # target directory
#         FILES_MATCHING PATTERN "*.h" # select header files
#         )

# install(EXPORT ${PROJECT_NAME}Config NAMESPACE ${PROJECT_NAME}:: DESTINATION share/${PROJECT_NAME}/cmake)
# export(TARGETS ${INSTALL_LIBS} FILE ${PROJECT_NAME}Config.cmake)