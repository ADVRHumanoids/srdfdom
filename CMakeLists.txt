cmake_minimum_required(VERSION 2.8.3)
project(srdfdom_advr)
list(APPEND CMAKE_MODULE_PATH "${CMAKE_CURRENT_LIST_DIR}/cmake")
include(MacroYCMInstallLibrary)

find_package(Boost REQUIRED)
include_directories(SYSTEM ${Boost_INCLUDE_DIR})

find_package(catkin REQUIRED COMPONENTS srdfdom)

include_directories(include ${catkin_INCLUDE_DIRS})
link_directories(${catkin_LIBRARY_DIRS})

add_library(${PROJECT_NAME} SHARED src/model.cpp)
target_link_libraries(${PROJECT_NAME} ${catkin_LIBRARIES})

ycm_library_install(${PROJECT_NAME} 1 0 0)
# install(TARGETS ${PROJECT_NAME}
#   DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
# )
# install(DIRECTORY include/${PROJECT_NAME}/
#   DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
#   FILES_MATCHING PATTERN "*.h"
# )

#######################
# Add Testing target  #
#######################
if(SRDFDOM_COMPILE_TESTS)
  enable_testing()
  add_subdirectory(test)
endif()
