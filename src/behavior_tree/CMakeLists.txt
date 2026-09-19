cmake_minimum_required(VERSION 3.16)
project(behavior_tree)

if(POLICY CMP0167)
  cmake_policy(SET CMP0167 NEW)
endif()

set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# =========================================
# 1. 查找依賴 (Find Packages)
# =========================================
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
endif()
get_filename_component(BTCPP_WS_SRC_DIR "${CMAKE_CURRENT_SOURCE_DIR}" DIRECTORY)
get_filename_component(BTCPP_WS_ROOT_DIR "${BTCPP_WS_SRC_DIR}" DIRECTORY)
set(BTCPP_V4_ROOT "${BTCPP_WS_ROOT_DIR}/third_party/behaviortree_cpp_v4/install"
  CACHE PATH "Install prefix of pinned BehaviorTree.CPP v4 (defaults to <ws>/third_party/behaviortree_cpp_v4/install)")
set(BTCPP_MIN_VERSION "4.0.0")
set(BTCPP_V4_CMAKE_DIR_SHARE "${BTCPP_V4_ROOT}/share/behaviortree_cpp/cmake")
set(BTCPP_V4_CMAKE_DIR_LIB "${BTCPP_V4_ROOT}/lib/cmake/behaviortree_cpp")
set(BTCPP_V4_CMAKE_PATHS "")
set(BTCPP_IS_PINNED FALSE)
set(BTCPP_SYSTEM_PREFIX_PATHS "")

if(EXISTS "${BTCPP_V4_CMAKE_DIR_SHARE}/behaviortree_cppConfig.cmake")
  list(APPEND BTCPP_V4_CMAKE_PATHS "${BTCPP_V4_CMAKE_DIR_SHARE}")
endif()
if(EXISTS "${BTCPP_V4_CMAKE_DIR_LIB}/behaviortree_cppConfig.cmake")
  list(APPEND BTCPP_V4_CMAKE_PATHS "${BTCPP_V4_CMAKE_DIR_LIB}")
endif()

set(BTCPP_FORCE_PINNED_DEFAULT OFF)
if(BTCPP_V4_CMAKE_PATHS)
  set(BTCPP_FORCE_PINNED_DEFAULT ON)
endif()
option(BTCPP_FORCE_PINNED "Force behavior_tree to use the workspace-pinned BehaviorTree.CPP copy" ${BTCPP_FORCE_PINNED_DEFAULT})

if(DEFINED ENV{AMENT_PREFIX_PATH} AND NOT "$ENV{AMENT_PREFIX_PATH}" STREQUAL "")
  string(REPLACE ":" ";" BTCPP_AMENT_PREFIX_PATHS "$ENV{AMENT_PREFIX_PATH}")
  list(APPEND BTCPP_SYSTEM_PREFIX_PATHS ${BTCPP_AMENT_PREFIX_PATHS})
endif()
if(DEFINED ENV{CMAKE_PREFIX_PATH} AND NOT "$ENV{CMAKE_PREFIX_PATH}" STREQUAL "")
  string(REPLACE ":" ";" BTCPP_CMAKE_PREFIX_PATHS "$ENV{CMAKE_PREFIX_PATH}")
  list(APPEND BTCPP_SYSTEM_PREFIX_PATHS ${BTCPP_CMAKE_PREFIX_PATHS})
endif()
if(DEFINED ENV{ROS_DISTRO} AND EXISTS "/opt/ros/$ENV{ROS_DISTRO}")
  list(APPEND BTCPP_SYSTEM_PREFIX_PATHS "/opt/ros/$ENV{ROS_DISTRO}")
endif()
list(APPEND BTCPP_SYSTEM_PREFIX_PATHS "/usr/local" "/usr")
list(REMOVE_DUPLICATES BTCPP_SYSTEM_PREFIX_PATHS)
list(REMOVE_ITEM BTCPP_SYSTEM_PREFIX_PATHS
  "${BTCPP_V4_ROOT}"
  "${BTCPP_V4_CMAKE_DIR_SHARE}"
  "${BTCPP_V4_CMAKE_DIR_LIB}")
set(BTCPP_SYSTEM_PREFIX_PATHS_EXISTING "")
foreach(BTCPP_PREFIX IN LISTS BTCPP_SYSTEM_PREFIX_PATHS)
  if(EXISTS "${BTCPP_PREFIX}")
    list(APPEND BTCPP_SYSTEM_PREFIX_PATHS_EXISTING "${BTCPP_PREFIX}")
  endif()
endforeach()
set(BTCPP_SYSTEM_PREFIX_PATHS "${BTCPP_SYSTEM_PREFIX_PATHS_EXISTING}")

# 旧 build 目录可能缓存了 fallback 的 third_party 路径；在“系统优先”模式下先清掉它，
# 避免 find_package 继续复用旧结果而跳过真正的系统搜索。
if(DEFINED behaviortree_cpp_DIR AND NOT behaviortree_cpp_DIR MATCHES "-NOTFOUND$")
  file(TO_CMAKE_PATH "${behaviortree_cpp_DIR}" BTCPP_CACHED_DIR_NORMALIZED)
  file(TO_CMAKE_PATH "${BTCPP_V4_ROOT}" BTCPP_V4_ROOT_NORMALIZED)
  string(FIND "${BTCPP_CACHED_DIR_NORMALIZED}" "${BTCPP_V4_ROOT_NORMALIZED}" BTCPP_CACHED_IS_PINNED_POS)
  if(BTCPP_CACHED_IS_PINNED_POS EQUAL 0)
    message(STATUS "BehaviorTree.CPP: clear cached pinned package dir to retry system search")
    unset(behaviortree_cpp_DIR CACHE)
    unset(behaviortree_cpp_FOUND)
  endif()
endif()

if(BTCPP_FORCE_PINNED AND NOT BTCPP_V4_CMAKE_PATHS)
  message(WARNING
    "BehaviorTree.CPP: BTCPP_FORCE_PINNED=ON but pinned workspace copy was not found.\n"
    "Retrying system/package-manager search instead.\n"
    "Expected pinned config in one of:\n"
    "  - ${BTCPP_V4_CMAKE_DIR_SHARE}\n"
    "  - ${BTCPP_V4_CMAKE_DIR_LIB}")
  set(BTCPP_FORCE_PINNED FALSE)
endif()

if(BTCPP_FORCE_PINNED)
  message(STATUS "BehaviorTree.CPP: force pinned workspace copy")
elseif(BTCPP_SYSTEM_PREFIX_PATHS)
  find_package(behaviortree_cpp ${BTCPP_MIN_VERSION} CONFIG QUIET
    PATHS ${BTCPP_SYSTEM_PREFIX_PATHS}
    NO_DEFAULT_PATH
  )
endif()

if(behaviortree_cpp_FOUND)
  message(STATUS "BehaviorTree.CPP source: system package v${BTCPP_MIN_VERSION}+ (preferred)")
  if(DEFINED behaviortree_cpp_DIR)
    message(STATUS "BehaviorTree.CPP config dir: ${behaviortree_cpp_DIR}")
  endif()
elseif(BTCPP_V4_CMAKE_PATHS)
  unset(behaviortree_cpp_DIR CACHE)
  if(BTCPP_FORCE_PINNED)
    find_package(behaviortree_cpp CONFIG REQUIRED
      PATHS ${BTCPP_V4_CMAKE_PATHS}
      NO_DEFAULT_PATH
    )
  else()
    find_package(behaviortree_cpp ${BTCPP_MIN_VERSION} CONFIG REQUIRED
      PATHS ${BTCPP_V4_CMAKE_PATHS}
      NO_DEFAULT_PATH
    )
  endif()
  set(BTCPP_IS_PINNED TRUE)
  message(STATUS "BehaviorTree.CPP source: pinned workspace v${BTCPP_MIN_VERSION}+ fallback")
  message(STATUS "Pinned BehaviorTree.CPP root: ${BTCPP_V4_ROOT}")
else()
  set(BTCPP_SYSTEM_PREFIX_PATHS_DISPLAY "(no system prefixes available)")
  if(BTCPP_SYSTEM_PREFIX_PATHS)
    string(REPLACE ";" "\n  - " BTCPP_SYSTEM_PREFIX_PATHS_DISPLAY "${BTCPP_SYSTEM_PREFIX_PATHS}")
    set(BTCPP_SYSTEM_PREFIX_PATHS_DISPLAY "  - ${BTCPP_SYSTEM_PREFIX_PATHS_DISPLAY}")
  endif()
  message(FATAL_ERROR
    "BehaviorTree.CPP v${BTCPP_MIN_VERSION}+ not found.\n"
    "Checked system/package-manager prefixes first:\n"
    "${BTCPP_SYSTEM_PREFIX_PATHS_DISPLAY}\n"
    "Checked pinned workspace paths as fallback:\n"
    "  - ${BTCPP_V4_CMAKE_DIR_SHARE}\n"
    "  - ${BTCPP_V4_CMAKE_DIR_LIB}\n"
    "System v3 installs are intentionally ignored.\n"
    "Install the system package, or install/copy BehaviorTree.CPP v4 into ${BTCPP_V4_ROOT}.")
endif()

if(NOT BTCPP_FORCE_PINNED AND (NOT DEFINED behaviortree_cpp_VERSION OR behaviortree_cpp_VERSION VERSION_LESS ${BTCPP_MIN_VERSION}))
  message(FATAL_ERROR
    "behavior_tree requires BehaviorTree.CPP v${BTCPP_MIN_VERSION}+, but found version '${behaviortree_cpp_VERSION}'.")
endif()

if(DEFINED behaviortree_cpp_VERSION AND NOT behaviortree_cpp_VERSION STREQUAL "")
  message(STATUS "BehaviorTree.CPP version: ${behaviortree_cpp_VERSION}")
elseif(BTCPP_FORCE_PINNED)
  message(WARNING "BehaviorTree.CPP version string unavailable from pinned config; using validated local library at ${BTCPP_V4_ROOT}")
else()
  message(FATAL_ERROR "BehaviorTree.CPP version string unavailable.")
endif()
if(TARGET behaviortree_cpp::behaviortree_cpp)
  set(BTCPP_LINK_TARGET behaviortree_cpp::behaviortree_cpp)
elseif(TARGET BT::behaviortree_cpp)
  set(BTCPP_LINK_TARGET BT::behaviortree_cpp)
else()
  message(FATAL_ERROR "No known BT target found. Expected behaviortree_cpp::behaviortree_cpp or BT::behaviortree_cpp.")
endif()
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)    # 根據你的要求保留
find_package(ament_index_cpp REQUIRED)  # 用於相對路徑

# 自定義消息包
find_package(auto_aim_common REQUIRED)
find_package(gimbal_driver REQUIRED)
find_package(sentry_msgs REQUIRED)

# 系統庫 (與 ROS 1 保持一致)
find_package(fmt REQUIRED)
find_package(Boost REQUIRED COMPONENTS system filesystem)

# =========================================
# 2. 添加子模組 (Logger)
# =========================================
# [關鍵修復] 根據 ROS 1 舊文件，這裡必須編譯 Logger
# 請確保你的專案根目錄下有 Logger 文件夾，且裡面有 CMakeLists.txt
if(EXISTS "${CMAKE_CURRENT_SOURCE_DIR}/Logger/CMakeLists.txt")
  add_subdirectory(Logger)
endif()

# =========================================
# 3. 編譯可執行檔 (Build Executable)
# =========================================
# 自動抓取 src 下所有 .cpp 以及根目錄的 main.cpp
file(GLOB_RECURSE SRC_FILES CONFIGURE_DEPENDS "src/*.cpp")

add_executable(${PROJECT_NAME}_node 
  main.cpp
  ${SRC_FILES}
)

add_executable(sentry_message_node
  sentry_message_main.cpp
  src/SentryMessageNode.cpp
)

if(BTCPP_IS_PINNED AND EXISTS "${BTCPP_V4_ROOT}/lib")
  set_target_properties(${PROJECT_NAME}_node PROPERTIES
    BUILD_RPATH "${BTCPP_V4_ROOT}/lib"
    INSTALL_RPATH "${BTCPP_V4_ROOT}/lib"
  )
  # ROS setup puts /opt/ros first in LD_LIBRARY_PATH. Keep the pinned headers
  # and library ABI paired instead of letting that environment override v4.
  target_link_options(${PROJECT_NAME}_node PRIVATE "-Wl,--disable-new-dtags")
endif()

target_compile_definitions(${PROJECT_NAME}_node PRIVATE LY_ENABLE_SENTRY_MSGS=1)

# 設置頭文件路徑
target_include_directories(${PROJECT_NAME}_node PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/module>
  ${Boost_INCLUDE_DIRS} # 顯式包含 Boost 頭文件路徑
)

# =========================================
# 4. 連結庫 (Link Libraries)
# =========================================
target_link_libraries(${PROJECT_NAME}_node
  fmt::fmt
  Boost::system
  Boost::filesystem
  ${BTCPP_LINK_TARGET}
  
  # [關鍵修復] 連結舊專案的 Logger 庫
  # 假設 Logger/CMakeLists.txt 裡定義的 library 名字叫 Utils::Logger
  Utils::Logger 
)

# 連結 ROS 2 相關依賴
ament_target_dependencies(${PROJECT_NAME}_node
  rclcpp
  std_msgs
  geometry_msgs
  auto_aim_common
  gimbal_driver
  ament_index_cpp
)

ament_target_dependencies(${PROJECT_NAME}_node sentry_msgs)

ament_target_dependencies(sentry_message_node
  rclcpp
  std_msgs
  gimbal_driver
  sentry_msgs
)
target_include_directories(sentry_message_node PRIVATE
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
)

if(BUILD_TESTING)
  ament_add_gtest(test_league3v3_policy
    test/test_league3v3_policy.cpp
    src/League3v3Policy.cpp
  )
  target_include_directories(test_league3v3_policy PRIVATE
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/module>
  )

  ament_add_gtest(test_outpost_engagement_lock
    test/test_outpost_engagement_lock.cpp
    src/OutpostEngagementLock.cpp
  )
  target_include_directories(test_outpost_engagement_lock PRIVATE
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/module>
  )
  ament_add_gtest(test_posture_manager
    test/test_posture_manager.cpp
    src/PostureManager.cpp
  )
  target_include_directories(test_posture_manager PRIVATE
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/module>
  )
  ament_add_gtest(test_regional_task_posture test/test_regional_task_posture.cpp)
  target_include_directories(test_regional_task_posture PRIVATE
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/module>
  )
  ament_add_gtest(test_aim_source test/test_aim_source.cpp)
  target_include_directories(test_aim_source PRIVATE
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/module>
  )
  ament_target_dependencies(test_aim_source gimbal_driver)
  ament_add_gtest(test_aim_config_policy test/test_aim_config_policy.cpp)
  target_include_directories(test_aim_config_policy PRIVATE
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  )
  ament_add_gtest(test_enemy_position_source_policy
    test/test_enemy_position_source_policy.cpp
  )
  target_include_directories(test_enemy_position_source_policy PRIVATE
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/module>
  )
  ament_add_gtest(test_external_aim_target_policy
    test/test_external_aim_target_policy.cpp
  )
  target_include_directories(test_external_aim_target_policy PRIVATE
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/module>
  )
  ament_add_gtest(test_decision_trace_control_output test/test_decision_trace_control_output.cpp)
  target_include_directories(test_decision_trace_control_output PRIVATE
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/module>
  )
  ament_target_dependencies(test_decision_trace_control_output
    rclcpp
    std_msgs
    geometry_msgs
    auto_aim_common
    gimbal_driver
    ament_index_cpp
  )
  target_link_libraries(test_decision_trace_control_output
    ${BTCPP_LINK_TARGET}
    Utils::Logger
  )
  ament_add_gtest(test_navi_rotate_posture test/test_navi_rotate_posture.cpp)
  target_include_directories(test_navi_rotate_posture PRIVATE
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/module>
  )
  ament_add_gtest(test_navi_speed_level test/test_navi_speed_level.cpp)
  target_include_directories(test_navi_speed_level PRIVATE
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/module>
  )
  ament_add_gtest(test_damage_rotate_policy test/test_damage_rotate_policy.cpp)
  target_include_directories(test_damage_rotate_policy PRIVATE
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/module>
  )
  ament_add_gtest(test_tactical_protection_policy test/test_tactical_protection_policy.cpp)
  target_include_directories(test_tactical_protection_policy PRIVATE
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/module>
  )
  ament_add_gtest(test_regional_defense_search_policy
    test/test_regional_defense_search_policy.cpp
  )
  target_include_directories(test_regional_defense_search_policy PRIVATE
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  )
  ament_add_gtest(test_decision_explain test/test_decision_explain.cpp)
  target_include_directories(test_decision_explain PRIVATE
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/module>
  )
  ament_add_gtest(test_face_mode_manager
    test/test_face_mode_manager.cpp
    src/FaceModeManager.cpp
    src/AreaManager.cpp
  )
  target_include_directories(test_face_mode_manager PRIVATE
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/module>
  )
  ament_target_dependencies(test_face_mode_manager
    rclcpp
    std_msgs
    gimbal_driver
  )
  ament_add_gtest(test_passive_gimbal_motion test/test_passive_gimbal_motion.cpp)
  target_include_directories(test_passive_gimbal_motion PRIVATE
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/module>
  )
  ament_add_gtest(test_ready_roadland_area test/test_ready_roadland_area.cpp)
  target_include_directories(test_ready_roadland_area PRIVATE
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/module>
  )
  ament_add_gtest(test_pre_ready_roadland_tasks
    test/test_pre_ready_roadland_tasks.cpp
    src/AreaManager.cpp
    src/DefaultStrategyManager.cpp
    src/EventManager.cpp
  )
  target_include_directories(test_pre_ready_roadland_tasks PRIVATE
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/module>
  )
  ament_target_dependencies(test_pre_ready_roadland_tasks
    rclcpp
    std_msgs
    gimbal_driver
  )
  ament_add_gtest(test_chase_policy
    test/test_chase_policy.cpp
    src/ChasePolicy.cpp
  )
  target_include_directories(test_chase_policy PRIVATE
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/module>
  )
  ament_add_gtest(test_near_goal_arrival_confirm
    test/test_near_goal_arrival_confirm.cpp
  )
  target_include_directories(test_near_goal_arrival_confirm PRIVATE
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/module>
  )
  ament_add_gtest(test_sentry_message_policy test/test_sentry_message_policy.cpp)
  target_include_directories(test_sentry_message_policy PRIVATE
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  )
endif()

add_executable(area_calculator
  tools/AreaCalculator.cpp
  src/AreaManager.cpp
)

target_include_directories(area_calculator PRIVATE
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/module>
)

ament_target_dependencies(area_calculator
  gimbal_driver
)

# =========================================
# 5. 安裝規則 (Install Rules)
# =========================================
# 安裝執行檔
install(TARGETS
  ${PROJECT_NAME}_node
  sentry_message_node
  area_calculator
  DESTINATION lib/${PROJECT_NAME}
)

# 安裝 Scripts (XML/Json)，讓 ament_index 能找到
if(EXISTS "${CMAKE_CURRENT_SOURCE_DIR}/Scripts")
  install(DIRECTORY Scripts
    DESTINATION share/${PROJECT_NAME}
  )
endif()

# 安裝 Launch 檔
if(EXISTS "${CMAKE_CURRENT_SOURCE_DIR}/launch")
  install(DIRECTORY launch
    DESTINATION share/${PROJECT_NAME}
  )
endif()

if(EXISTS "${BTCPP_WS_ROOT_DIR}/config")
  install(DIRECTORY "${BTCPP_WS_ROOT_DIR}/config/"
    DESTINATION share/${PROJECT_NAME}/config
  )
endif()

if(EXISTS "${CMAKE_CURRENT_SOURCE_DIR}/config")
  install(DIRECTORY config/
    DESTINATION share/${PROJECT_NAME}/config
  )
endif()

# 安裝 Include (如果需要給別人用)
if(EXISTS "${CMAKE_CURRENT_SOURCE_DIR}/include")
  install(DIRECTORY include/
    DESTINATION include
  )
endif()

# =========================================
# 6. 導出依賴 (Export Dependencies)
# =========================================
ament_export_dependencies(
  rclcpp 
  behaviortree_cpp 
  std_msgs 
  geometry_msgs
  auto_aim_common 
  gimbal_driver
  ament_index_cpp
)

ament_package()
