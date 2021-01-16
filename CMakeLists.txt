#################### BASIC SETUP ####################
cmake_minimum_required(VERSION 3.11)
project(iekf VERSION 0.1 LANGUAGES CXX)

###############################################################################
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

###############################################################################
# Import necessary external dependencies

# Eigen 
find_package(Eigen3 REQUIRED)


###############################################################################
add_subdirectory(src)
add_subdirectory(exe)