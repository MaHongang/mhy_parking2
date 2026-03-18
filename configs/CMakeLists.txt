# configs 模块包含配置文件头文件和实现文件
# open_space_trajectory_optimizer_config 有 .cc 实现文件
add_library(configs STATIC
    open_space_trajectory_optimizer_config.cc
)

find_package(Eigen3 REQUIRED)

target_include_directories(configs PUBLIC
                            ${PROJECT_SOURCE_DIR}#工程目录
                            "usr/include"#系统目录
                            "usr/local/include"#安装文件目录
)

target_link_libraries(configs PUBLIC
    Eigen3::Eigen
)