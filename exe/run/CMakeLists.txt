cmake_minimum_required(VERSION 3.11)

set(SRCS "run.cpp")

# Create Executable
set(EXE_NAME run)
add_executable(${EXE_NAME} ${SRCS})

target_include_directories(${EXE_NAME} PUBLIC
  $<BUILD_INTERFACE:${PROJECT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
  )  
target_compile_features(${EXE_NAME} PUBLIC cxx_std_11)
target_link_libraries(${EXE_NAME} PUBLIC Eigen3::Eigen iekf SE2_3_Bias)