cmake_minimum_required(VERSION 3.16)   # Unity build と PCH を使うために 3.16 以上を推奨
project(cooperative_transportation_4ws_description)

# ================= コンパイラ/グローバル設定 =================
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

# ビルドタイプ（catkin 側指定がなければ Debug を既定に）
if(NOT CMAKE_BUILD_TYPE)
  set(CMAKE_BUILD_TYPE Debug CACHE STRING "Build type" FORCE)
endif()

# 共有/静的混在でも安全なように、全ターゲットを PIC 化
set(CMAKE_POSITION_INDEPENDENT_CODE ON)

# ccache（あれば自動で有効化）
find_program(CCACHE_PROGRAM ccache)
if(CCACHE_PROGRAM)
  message(STATUS "ccache found: enabling compiler launcher")
  set(CMAKE_CXX_COMPILER_LAUNCHER ${CCACHE_PROGRAM})
endif()

# ================= 依存 =================
find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
  tf
  tf2
  tf2_ros
  gazebo_plugins
)

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES kinematics_solver mathematica
  CATKIN_DEPENDS roscpp rospy std_msgs tf tf2 tf2_ros gazebo_plugins
)

# ================= インクルード =================
include_directories(
  ${catkin_INCLUDE_DIRS}
  ${PROJECT_SOURCE_DIR}/include
  /usr/include/eigen3
)

# ================= 自動生成(200+) → “mathematica” ライブラリ =================
file(GLOB_RECURSE MATHEMATICA_SRCS
     "${CMAKE_CURRENT_SOURCE_DIR}/src/mathematica/*.cpp")

# 静的でも PIC 必須（共有に取り込むため）
add_library(mathematica STATIC ${MATHEMATICA_SRCS})
set_target_properties(mathematica PROPERTIES
  POSITION_INDEPENDENT_CODE ON
  UNITY_BUILD ON                      # ★ Unity/Jumbo ビルドでコンパイル回数を削減
  UNITY_BUILD_BATCH_SIZE 24           # 適宜調整（大きくするとメモリ消費↑）
)

target_include_directories(mathematica PUBLIC
  ${PROJECT_SOURCE_DIR}/include
  ${catkin_INCLUDE_DIRS}
)

# 生成コードはデバッグ情報を軽く/無しにしてビルド時間短縮
# （gdbで生成関数に入らないなら -g0 推奨、多少残すなら -g1）
target_compile_options(mathematica PRIVATE -O2 -g0)

# PCH（前処理を共有）：生成コードが大量に Eigen を読むなら効く
# CMake 3.16+ / GCC/Clang 対応
target_precompile_headers(mathematica PRIVATE
  <Eigen/Core>
  <Eigen/Dense>
  <vector>
  <array>
  <cmath>
)

target_link_libraries(mathematica
  ${catkin_LIBRARIES}
)

# ================= ソルバ本体（共有ライブラリに明示） =================
# 以前のリンクエラーは libkinematics_solver.so 作成時に発生していたので SHARED を明示
add_library(kinematics_solver SHARED
  src/kinematics_solver.cpp
)
set_target_properties(kinematics_solver PROPERTIES
  POSITION_INDEPENDENT_CODE ON
)

target_include_directories(kinematics_solver PUBLIC
  ${PROJECT_SOURCE_DIR}/include
  ${catkin_INCLUDE_DIRS}
)

target_link_libraries(kinematics_solver
  mathematica
  ${catkin_LIBRARIES}
)

# ================= 既存ノード =================

add_executable(cooperative_transportation_kinematics_controller
  src/cooperative_transportation_kinematics.cpp
  src/vehicle_ct.cpp
  src/callback_ct.cpp
  src/csvLogger.cpp
  src/Bezier.cpp
  src/differential_equations.cpp
  src/getInputValue.cpp
  src/mathFunc.cpp
  src/DynamicsCalc.cpp
  src/initial.cpp
)

target_link_libraries(cooperative_transportation_kinematics_controller
kinematics_solver
  ${catkin_LIBRARIES}
)

add_executable(cooperative_transportation_dynamics_controller
  src/cooperative_transportation_dynamics.cpp
  src/vehicle_ct.cpp
  src/callback_dy.cpp
  src/csvLogger.cpp
  src/Bezier.cpp
  src/differential_equations.cpp
  src/getInputValue.cpp
  src/mathFunc.cpp
  src/DynamicsCalc.cpp
  src/initial.cpp
  src/dynamics_integrator_matrix.cpp
)

target_link_libraries(cooperative_transportation_dynamics_controller
  kinematics_solver
  ${catkin_LIBRARIES}
)

# （任意）mathematica のみ最適化レベルを強めたい場合（RelWithDebInfo 相当）
# if(CMAKE_BUILD_TYPE STREQUAL "Debug")
#   target_compile_options(mathematica PRIVATE -O2 -g0)
# endif()