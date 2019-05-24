set(the_description "3D point features")

#设定版本
cmake_minimum_required( VERSION 2.8 )

#设定工程名
project( PPF )

#设定可执行二进制文件的目录
SET( EXECUTABLE_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/bin) 

#增加子文件夹，也就是进入源代码文件夹继续构建
ADD_SUBDIRECTORY( ${PROJECT_SOURCE_DIR}/src)
