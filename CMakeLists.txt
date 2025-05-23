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
option(EXAMPLES "Build Examples"        OFF)
option(PYTHON   "Build Python Wrapper"  OFF)
option(TESTS    "Build Tests"           OFF)

# Eigen 
find_package(Eigen3 CONFIG QUIET)
if(NOT Eigen3_FOUND)
  message(STATUS "Pulling Eigen3 from git...")
  FetchContent_Declare(
      Eigen3
      GIT_REPOSITORY https://gitlab.com/libeigen/eigen.git
      GIT_TAG        3.4.0
  )
  option(EIGEN_BUILD_DOC OFF)
  option(BUILD_TESTING OFF)
  option(EIGEN_LEAVE_TEST_IN_ALL_TARGET OFF)
  option(EIGEN_BUILD_PKGCONFIG OFF)
  FetchContent_MakeAvailable(Eigen3)
endif()

#################### ADD TESTS ####################
if(TESTS)
  # Google Test
  FetchContent_Declare(
    googletest
    GIT_REPOSITORY https://github.com/google/googletest.git
    GIT_TAG        origin/main
  )
  option(INSTALL_GTEST OFF)
  FetchContent_MakeAvailable(googletest)

    
  enable_testing()
  add_subdirectory(tests)
endif()  

#################### ADD TARGETS ####################
add_subdirectory(src)

if(EXAMPLES)
  add_subdirectory(examples)
endif()

if(PYTHON)
  add_subdirectory(python)
endif()


#################### INSTALL AS LIBRARY ####################
set(INSTALL_LIBS Core SE2Models Inertial)
install(TARGETS ${INSTALL_LIBS} EXPORT ${PROJECT_NAME}Config
        ARCHIVE DESTINATION lib
        LIBRARY DESTINATION lib
        RUNTIME DESTINATION bin)

install(DIRECTORY "${CMAKE_SOURCE_DIR}/include" # source directory
        DESTINATION "${CMAKE_INSTALL_PREFIX}" # target directory
        )

install(EXPORT ${PROJECT_NAME}Config NAMESPACE ${PROJECT_NAME}:: DESTINATION share/${PROJECT_NAME}/cmake)
export(TARGETS ${INSTALL_LIBS} FILE ${PROJECT_NAME}Config.cmake)

# And uninstall option
add_custom_target("remove" COMMENT "Remove installed files")
add_custom_command(
    TARGET "remove"
    POST_BUILD
    COMMENT "Uninstall files with install_manifest.txt"
    COMMAND xargs rm -vf < install_manifest.txt || echo Nothing in
            install_manifest.txt to be removeed!
)