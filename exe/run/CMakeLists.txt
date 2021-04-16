cmake_minimum_required(VERSION 3.11)

find_package(Python3 COMPONENTS Development NumPy)

set(SRCS "run.cpp")

# Create Executable
set(EXE_NAME run)
add_executable(${EXE_NAME} ${SRCS})

target_include_directories(${EXE_NAME} PUBLIC
  $<BUILD_INTERFACE:${PROJECT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
  ${Python3_INCLUDE_DIRS} ${Python3_NumPy_INCLUDE_DIRS}
  )  
target_compile_features(${EXE_NAME} PUBLIC cxx_std_17)
target_link_libraries(${EXE_NAME} PUBLIC Eigen3::Eigen Core SE2_SLAM Python3::Python Python3::NumPy)