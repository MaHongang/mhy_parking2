aux_source_directory(./path  PATH_SRC )
aux_source_directory(./speed SPEED_SRC )
aux_source_directory(./piecewise_jerk PIECE_SRC)
aux_source_directory(./trajectory TRAJ_SRC )

add_library(planning_data ${PATH_SRC} ${SPEED_SRC} ${PIECE_SRC} ${TRAJ_SRC})

# osqp 是必需依赖，用于分段 jerk 速度优化
find_package(osqp REQUIRED)

target_include_directories(planning_data PUBLIC 
                            ${PROJECT_SOURCE_DIR}  # 工程目录
                            "usr/include"          # 系统目录
                            "usr/local/include"    # 安装文件目录
)

target_link_libraries(planning_data PUBLIC
                    common_math
                    osqp::osqp
)
