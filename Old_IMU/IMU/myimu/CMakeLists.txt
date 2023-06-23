cmake_minimum_required(VERSION 3.10)

project(myimu)

set(CMAKE_MODULE_PATH ${CMAKE_CURRENT_SOURCE_DIR}/cmake/)
find_package(Freespace)

## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(
  ${Freespace_INCLUDES}
)

set(APP_CTRL_SRCS src/appControlHandler.c src/appControlHandler.h)
set(MATH_SRCS src/math/quaternion.c src/math/quaternion.h src/math/vec3.h)


set(THREADS_PREFER_PTHREAD_FLAG ON)
find_package(Threads REQUIRED)

set (ALL_SRC
    ${MATH_SRC}
)

add_executable(${PROJECT_NAME}_exec src/myimu.cpp)

##target_link_libraries(${PROJECT_NAME}_exec PRIVATE )
target_link_libraries(${PROJECT_NAME}_exec
  ${Freespace_LIBRARIES}
  Threads::Threads
)
