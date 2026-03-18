cmake_minimum_required(VERSION 3.10)
project(mhy_parking2)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# Find OpenMP for parallel computing
find_package(OpenMP REQUIRED)

# Find Abseil for string utilities
find_package(absl REQUIRED)

# ==================== Add submodules ====================
# common/math/       -> common_math      (数学工具库：vec2d, box2d, polygon2d 等)
# common/matplot/    -> matplotlib_cpp   (C++ matplotlib 绑定，INTERFACE 库)
# common/util/       -> (头文件)
# configs/           -> configs          (车辆和规划器配置，INTERFACE 库)
# planning_data/     -> planning_data    (轨迹数据结构)
# coarse_trajectory_generator/ -> coarse_trajectory_generator (Hybrid A*, RS 曲线等)
# open_space_map/    -> open_space_map   (地图处理和可视化)
# trajectory_smoother/ -> trajectory_smoother (轨迹平滑优化)
add_subdirectory(common)
add_subdirectory(configs)
add_subdirectory(planning_data)
add_subdirectory(coarse_trajectory_generator)
add_subdirectory(open_space_map)
add_subdirectory(trajectory_smoother)

# ==================== Reed-Shepp 独立库（用于测试） ====================
# 仅包含 RS 曲线相关的源文件，避免 hybrid_a_star 的额外依赖
add_library(reeds_shepp_lib STATIC
    coarse_trajectory_generator/reeds_shepp_path.cc
    coarse_trajectory_generator/node3d.cc
)
target_include_directories(reeds_shepp_lib PUBLIC ${PROJECT_SOURCE_DIR})
target_link_libraries(reeds_shepp_lib PUBLIC 
    common_math 
    configs 
    OpenMP::OpenMP_CXX
)

# ==================== Grid Search 独立库（用于测试） ====================
# 仅包含 Grid A* 搜索相关的源文件
add_library(grid_search_lib STATIC
    coarse_trajectory_generator/grid_search.cc
)
target_include_directories(grid_search_lib PUBLIC ${PROJECT_SOURCE_DIR})
target_link_libraries(grid_search_lib PUBLIC 
    common_math 
    configs 
    absl::strings
)

# ==================== Test Executables ====================

# Test: Reed-Shepp curve generation (commented out - test files may not exist)
# add_executable(test_reeds_shepp test/test_reeds_shepp.cpp)
# target_link_libraries(test_reeds_shepp 
#     reeds_shepp_lib
#     matplotlib_cpp
#     OpenMP::OpenMP_CXX
# )

# Test: Grid A* search algorithm (commented out - test files may not exist)
# add_executable(test_grid_search test/test_grid_search.cpp)
# target_link_libraries(test_grid_search 
#     grid_search_lib
#     matplotlib_cpp
# )

# Test: Hybrid A* path planning algorithm (commented out - test files may not exist)
# add_executable(test_hybrid_a_star test/test_hybrid_a_star.cpp)
# target_link_libraries(test_hybrid_a_star 
#     coarse_trajectory_generator
#     matplotlib_cpp
# )

# ==================== Safe Corridor Trajectory Optimizer ====================
# Build safe corridor library
add_library(safe_corridor_optimizer STATIC
    safe_corridor_trajectory_optimizer/open_space_trajectory_optimizer.cc
    safe_corridor_trajectory_optimizer/construct_driving_corridor.cc
)

find_package(Eigen3 REQUIRED)
find_package(Python3 COMPONENTS Development NumPy REQUIRED)

target_include_directories(safe_corridor_optimizer PUBLIC 
    ${PROJECT_SOURCE_DIR}
    ${Python3_INCLUDE_DIRS}
    ${Python3_NumPy_INCLUDE_DIRS}
)

target_link_libraries(safe_corridor_optimizer PUBLIC
    configs
    common_math
    planning_data
    coarse_trajectory_generator
    open_space_map
    trajectory_smoother
    Eigen3::Eigen
    Python3::Python
    Python3::NumPy
)

# Test: Safe Corridor parking (commented out - file not found)
# add_executable(test_corridor_parking safe_corridor_trajectory_optimizer/corridor_parking.cc)
# target_link_libraries(test_corridor_parking 
#     safe_corridor_optimizer
#     matplotlib_cpp
# )

# Test: Safe Corridor parking test
add_executable(test_safe_corridor_parking 
    safe_corridor_trajectory_optimizer/safe_corridor_parking_test.cc)
target_link_libraries(test_safe_corridor_parking 
    safe_corridor_optimizer
    matplotlib_cpp
    Python3::Python
    Python3::NumPy
)

# ==================== OBCA Trajectory Optimizer ====================
# Add OBCA subdirectory (commented out if directory doesn't exist)
# add_subdirectory(OBCA_trajectory_optimizer)

# Test: OBCA Apollo parking (commented out if directory doesn't exist)
# add_executable(test_apollo_parking OBCA_trajectory_optimizer/apollo_parking.cc)
# target_link_libraries(test_apollo_parking 
#     obca_trajectory_optimizer
#     open_space_map
#     matplotlib_cpp
# )
