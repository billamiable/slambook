# 习题解答

1、阅读文献[1]和[14]，你能看懂文献的内容吗？

2、阅读SLAM的综述文献,例如[9,15,16,17,18]等。这些文献关于SLAM的看法与本书有何异同？

3、g++命令有哪些参数？怎么填写参数可以更改生成的程序文件名？

g++编译原理：https://blog.csdn.net/elfprincexu/article/details/45043971

gcc/g++在执行编译工作的时候，总共需要4步：

- 预处理：主要处理#include和#define，它把#include包含进来的.h 文件插入到#include所在的位置，把源程序中使用到的用#define定义的宏用实际的字符串代替，生成.i的文件
- 编译：gcc首先要检查代码的规范性、是否有语法错误等，以确定代码的实际要做的工作，在检查无误后，gcc把代码翻译成汇编语言，生成.s文件
- 汇编：把.s文件翻译成二进制机器指令文件.o
- 链接：gcc按照路径查找函数库，完成链接后生成可执行文件

g++命令参数：https://www.jianshu.com/p/40ac8313eb7f，https://www.cnblogs.com/lidan/archive/2011/05/25/2239517.html

重点关注以下几个参数：

- -E：只做预处理。
- -S：只做预处理和编译。
- -c ：只做预处理、编译和汇编。
- -o：制定目标名称，缺省的时候，gcc 编译出来的文件是a.out。
- -llibrary：制定编译的时候使用的库，例如gcc -lcurses hello.c表示使用ncurses库。
-  -Ldir：制定编译的时候，搜索库的路径。

4、使用build文件夹来编译你的cmake工程，然后在KDevelop中试试

5、刻意在代码中添加一些语法错误，看看编译会生成什么样的信息。你能看懂g++的错误吗？

6、如果忘了把库链接到可执行程序上，编译会报错吗？什么样的错？

```terminal
Undefined symbols for architecture x86_64:
  "printHello()", referenced from:
      _main in useHello.cpp.o
ld: symbol(s) not found for architecture x86_64
clang: error: linker command failed with exit code 1 (use -v to see invocation)
make[2]: *** [useHello] Error 1
make[1]: *** [CMakeFiles/useHello.dir/all] Error 2
make: *** [all] Error 2
```
表示链接失败，找不到对应的symbol

7、阅读《cmake实践》,了解cmake的其他语法。

8、完善hello SLAM的小程序，把它做成一个小程序库，安装到本地硬盘中。然后，新建一个工程，使用             找这个库并调用它。

安装到本地硬盘方法：
修改CMakeLists.txt，并重新cmake & make & make install

```c
# 声明要求的 cmake 最低版本
cmake_minimum_required( VERSION 2.8 )

# 声明一个 cmake 工程
project( HelloSLAM )

# 设置编译模式
set( CMAKE_BUILD_TYPE "Debug" )
set(CMAKE_INSTALL_PREFIX /usr/local)

# 添加一个可执行程序
# 语法：add_executable( 程序名 源代码文件 ）
add_executable( helloSLAM helloSLAM.cpp )

# 添加一个库
add_library( hello libHelloSLAM.cpp )
install(TARGETS hello
    DESTINATION ${CMAKE_INSTALL_PREFIX}/hello/lib
    )
install(FILES libHelloSLAM.h DESTINATION ${CMAKE_INSTALL_PREFIX}/hello/include)
# 共享库
# add_library( hello_shared SHARED libHelloSLAM.cpp )

add_executable( useHello useHello.cpp )

# 将库文件链接到可执行程序上
# target_link_libraries( useHello hello_shared )
target_link_libraries( useHello hello )
```

新建一个project后，创建cmake_modules目录，新建FindHELLO.cmake文件，用于找到本地硬盘中的hello头文件和库文件：

```c
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
```

最后修改新的project里的CMakeLists.txt，并再次编译运行程序即可：

```c
# 声明要求的 cmake 最低版本
cmake_minimum_required( VERSION 2.8 )

# 声明一个 cmake 工程
project( useHelloSLAM )

# 设置编译模式
set( CMAKE_BUILD_TYPE "Debug" )

# 添加一个可执行程序
# 语法：add_executable( 程序名 源代码文件 ）
add_executable( usehelloSLAM use_helloSLAM.cpp )

# 寻找
set(CMAKE_MODULE_PATH ${PROJECT_SOURCE_DIR}/cmake_modules)
find_package(HELLO)
include_directories(${HELLO_INCLUDE_DIRS})
target_link_libraries (usehelloSLAM ${HELLO_LIBRARIES})
```

以上代码具体可见本github中ch2的new_project目录。

9、寻找其他cmake教学材料，深入了解cmake，例如https://github.com/TheErk/CMake-tutorial。

10、寻找Kdevelop的官方网站，看看它还有哪些特性。你都用上了吗？

11、如果你在上一讲学习了vim，请试试Kdevelop的vim编辑功能。                       





# 参考文献

- g++参数：https://www.jianshu.com/p/40ac8313eb7f
- g++参数：https://www.cnblogs.com/lidan/archive/2011/05/25/2239517.html
- g++编译原理：https://blog.csdn.net/elfprincexu/article/details/45043971