aux_source_directory(. SMOOTHER_SRC)

#link_directories的位置得放到add_executable前面
link_directories(
    "usr/local/lib"
    "usr/local/lib64"
    "usr/lib"
    ${PROJECT_BINARY_DIR}/lib
)

add_library(trajectory_smoother ${SMOOTHER_SRC})

# Find required packages
find_package(Eigen3 REQUIRED)
find_package(OpenMP REQUIRED)

target_include_directories(trajectory_smoother PUBLIC 
                            ${PROJECT_SOURCE_DIR}#工程目录
                            "usr/include"#系统目录
                            "usr/local/include"#安装文件目录
)

# Add OpenMP compile flags
target_compile_options(trajectory_smoother PUBLIC ${OpenMP_CXX_FLAGS})

# Define HAVE_STDDEF_H for IPOPT compatibility
target_compile_definitions(trajectory_smoother PUBLIC HAVE_STDDEF_H)

target_link_libraries(trajectory_smoother PUBLIC
    configs 
    common_math
    Eigen3::Eigen
    OpenMP::OpenMP_CXX
    ipopt
)

# Try to find ADOLC library
find_library(ADOLC_LIB adolc PATHS /usr/lib /usr/local/lib /usr/local/lib64)
if(ADOLC_LIB)
    target_link_libraries(trajectory_smoother PUBLIC ${ADOLC_LIB})
else()
    # Fallback to hardcoded path if not found
    if(EXISTS /usr/local/lib64/libadolc.so.2)
        target_link_libraries(trajectory_smoother PUBLIC /usr/local/lib64/libadolc.so.2)
    elseif(EXISTS /usr/lib/x86_64-linux-gnu/libadolc.so)
        target_link_libraries(trajectory_smoother PUBLIC /usr/lib/x86_64-linux-gnu/libadolc.so)
    else()
        message(WARNING "ADOLC library not found, trajectory_smoother may fail to link")
    endif()
endif()

# Find and link glog library
find_library(GLOG_LIB glog PATHS /usr/lib /usr/local/lib /usr/lib/x86_64-linux-gnu)
if(GLOG_LIB)
    target_link_libraries(trajectory_smoother PUBLIC ${GLOG_LIB})
else()
    message(WARNING "glog library not found, some features may not work")
endif()




