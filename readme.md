todo:matplotlibcpp.h中两个头文件无法检索到，可以使用cmake来find_package(Python3 COMPONENTS Interpreter Development NumPy REQUIRED)来，更简洁
c_cpp_properties.json添加
"/usr/include/python3.10",
                "/usr/lib/python3/dist-packages/numpy/core/include"
来解决