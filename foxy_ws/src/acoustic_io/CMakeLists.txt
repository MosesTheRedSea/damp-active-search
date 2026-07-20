cmake_minimum_required(VERSION 3.8)
project(acoustic_io)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(PkgConfig REQUIRED)

pkg_check_modules(PORTAUDIO REQUIRED portaudio-2.0)
pkg_check_modules(SNDFILE REQUIRED sndfile)
pkg_check_modules(FFTW3 REQUIRED fftw3)

set(dependencies
  rclcpp
  std_msgs
)

add_executable(acoustic src/acoustic.cpp)

target_include_directories(acoustic PRIVATE
    ${PORTAUDIO_INCLUDE_DIRS}
    ${SNDFILE_INCLUDE_DIRS}
    ${FFTW3_INCLUDE_DIRS}
)

target_link_libraries(acoustic
    ${PORTAUDIO_LIBRARIES}
    ${SNDFILE_LIBRARIES}
    ${FFTW3_LIBRARIES}
)

ament_target_dependencies(acoustic ${dependencies})

install(TARGETS
  acoustic
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY launch config
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
