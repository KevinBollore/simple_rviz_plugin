cmake_minimum_required(VERSION 2.8.3)
project(simple_rviz_plugin)

find_package(catkin REQUIRED COMPONENTS
             simple_node
             rviz)

catkin_package()

include_directories(${catkin_INCLUDE_DIRS})
link_directories(${catkin_LIBRARY_DIRS})

find_package(Qt4 COMPONENTS QtCore QtGui REQUIRED)
include(${QT_USE_FILE})

add_definitions(-DQT_NO_KEYWORDS)

qt4_wrap_cpp(MOC_FILES
  src/simple_panel_widget.h
  src/x_widget.h
  src/y_widget.h
  src/z_widget.h
)

set(SOURCE_FILES
  src/simple_panel_widget.cpp
  src/x_widget.cpp
  src/y_widget.cpp
  src/z_widget.cpp 
  ${MOC_FILES}
)

add_library(${PROJECT_NAME} ${SOURCE_FILES})
set_property(TARGET ${PROJECT_NAME} APPEND_STRING PROPERTY COMPILE_FLAGS -Wall)
target_link_libraries(${PROJECT_NAME} ${QT_LIBRARIES} ${catkin_LIBRARIES})

## Install rules
install(TARGETS
  ${PROJECT_NAME}
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(FILES 
  plugin_description.xml
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})

