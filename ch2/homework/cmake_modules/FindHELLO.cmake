# 这里相当于告诉电脑如何查找HELLO
# 辅助输出信息
message("now using FindHELLO.cmake find hello lib")

# 将libHelloSLAM.h文件路径赋值给HELLO_INCLUDE_DIRS
FIND_PATH(HELLO_INCLUDE_DIRS libHelloSLAM.h /usr/local/hello/include)
message("./h dir ${HELLO_INCLUDE_DIRS}")

# 将libhello.a文件路径赋值给HELLO_LIBRARIES
FIND_LIBRARY(HELLO_LIBRARIES libhello.a /usr/local/hello/lib)
message("lib dir: ${HELLO_LIBRARIES}")

if(HELLO_INCLUDE_DIRS AND HELLO_LIBRARIES)
    # 设置变量结果
    set(HELLO_FOUND TRUE)
endif(HELLO_INCLUDE_DIRS AND HELLO_LIBRARIES)