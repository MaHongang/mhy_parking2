# aux_source_directory(. MAP_SRC)

# 查找Python3
find_package(Python3 COMPONENTS Development NumPy REQUIRED)

add_library(open_space_map open_space_map.cc)
find_package(Eigen3)
target_include_directories(open_space_map PUBLIC 
                            ${PROJECT_SOURCE_DIR}#工程目录
                            "usr/include"#系统目录
                            "usr/local/include"#安装文件目录
                            ${Python3_INCLUDE_DIRS}#python目录
                            ${Python3_NumPy_INCLUDE_DIRS}#numpy目录
)


target_link_libraries(open_space_map PUBLIC
    common_math
    matplotlib_cpp
    planning_data
    Eigen3::Eigen
    Python3::Python
    Python3::NumPy
)


# matplot_test has been moved to common/matplot/matplot_test.cc
# Use 'make test_matplot' to build the independent test
#add_executable(matplot_test matplot_test.cc)
#target_link_libraries(matplot_test PRIVATE matplotlib_cpp)