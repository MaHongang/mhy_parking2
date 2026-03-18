# util 模块是 header-only 库（只包含头文件）
# 使用 INTERFACE 库类型，不编译任何源文件，仅提供 include 路径

add_library(common_util INTERFACE)

# INTERFACE 库只需要指定 include 目录，供其他目标使用
target_include_directories(common_util INTERFACE 
    ${PROJECT_SOURCE_DIR}
)
