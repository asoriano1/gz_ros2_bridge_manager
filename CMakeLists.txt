cmake_minimum_required(VERSION 3.16)
project(gz_ros2_bridge_manager CXX)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
  set(CMAKE_CXX_STANDARD_REQUIRED ON)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)

# ---- Gazebo Harmonic ----
find_package(gz-gui8        REQUIRED)
find_package(gz-transport13 REQUIRED)
find_package(gz-msgs10      REQUIRED)

# ---- Qt 5 ----
if(POLICY CMP0100)
  cmake_policy(SET CMP0100 NEW)
endif()
set(CMAKE_AUTOMOC ON)
set(CMAKE_AUTORCC ON)
find_package(Qt5 REQUIRED COMPONENTS Core Concurrent Gui Quick QuickControls2)

# ===================================================================
# Plugin target
# Class:   Ros2BridgeManagerGui  →  libRos2BridgeManagerGui.so
# QML res: :/Ros2BridgeManagerGui/Ros2BridgeManagerGui.qml
# ===================================================================
set(PLUGIN_NAME Ros2BridgeManagerGui)

set(PLUGIN_SOURCES
  src/Ros2BridgeManagerGui.cpp
  src/BridgeTypeMapper.cpp
  src/WorldDiscovery.cpp
  src/GazeboTopicDiscovery.cpp
  src/ModelSensorDiscovery.cpp
  src/TopicAssociationHeuristic.cpp
  src/BridgeCommandBuilder.cpp
  src/BridgeSession.cpp
  src/ModelTopicSelectionStore.cpp
)

set(PLUGIN_MOC_HEADERS
  include/gz_ros2_bridge_manager/Ros2BridgeManagerGui.hh
)

qt5_add_resources(PLUGIN_RCC Ros2BridgeManagerGui.qrc)

add_library(${PLUGIN_NAME} SHARED
  ${PLUGIN_SOURCES}
  ${PLUGIN_MOC_HEADERS}
  ${PLUGIN_RCC}
)

target_include_directories(${PLUGIN_NAME}
  PRIVATE
    ${CMAKE_CURRENT_SOURCE_DIR}/include
)

target_link_libraries(${PLUGIN_NAME}
  PRIVATE
    gz-gui8::gz-gui8
    gz-transport13::gz-transport13
    gz-msgs10::gz-msgs10
    Qt5::Core
    Qt5::Concurrent
    Qt5::Gui
    Qt5::Quick
    Qt5::QuickControls2
)

# ---- Install ----
install(TARGETS ${PLUGIN_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

install(DIRECTORY gui/
  DESTINATION share/${PROJECT_NAME}/gui
  FILES_MATCHING PATTERN "*.qml"
)

install(DIRECTORY config/
  DESTINATION share/${PROJECT_NAME}/config
  OPTIONAL
)

ament_environment_hooks(
  "${CMAKE_CURRENT_SOURCE_DIR}/env-hooks/gz_ros2_bridge_manager.sh.in"
)

# ---- Tests ----
if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)

  ament_add_gtest(test_bridge_type_mapper
    test/test_bridge_type_mapper.cpp
    src/BridgeTypeMapper.cpp
  )
  target_include_directories(test_bridge_type_mapper
    PRIVATE ${CMAKE_CURRENT_SOURCE_DIR}/include
  )

  ament_add_gtest(test_topic_association_heuristic
    test/test_topic_association_heuristic.cpp
    src/TopicAssociationHeuristic.cpp
    src/BridgeTypeMapper.cpp
  )
  target_include_directories(test_topic_association_heuristic
    PRIVATE ${CMAKE_CURRENT_SOURCE_DIR}/include
  )

  ament_add_gtest(test_bridge_command_builder
    test/test_bridge_command_builder.cpp
    src/BridgeCommandBuilder.cpp
  )
  target_include_directories(test_bridge_command_builder
    PRIVATE ${CMAKE_CURRENT_SOURCE_DIR}/include
  )

  ament_add_gtest(test_model_topic_selection_store
    test/test_model_topic_selection_store.cpp
    src/ModelTopicSelectionStore.cpp
  )
  target_include_directories(test_model_topic_selection_store
    PRIVATE ${CMAKE_CURRENT_SOURCE_DIR}/include
  )

  ament_add_gtest(test_bridge_session
    test/test_bridge_session.cpp
    src/BridgeSession.cpp
    src/BridgeCommandBuilder.cpp
    src/ModelTopicSelectionStore.cpp
    src/TopicAssociationHeuristic.cpp
    src/BridgeTypeMapper.cpp
  )
  target_include_directories(test_bridge_session
    PRIVATE ${CMAKE_CURRENT_SOURCE_DIR}/include
  )
endif()

ament_package()
