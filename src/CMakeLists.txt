# 添加c++ 11标准支持
set( CMAKE_CXX_FLAGS "-std=c++11" )

# 寻找OpenCV库
set( OpenCV_DIR /usr/local/opencv34/share/OpenCV )

find_package( OpenCV 3 REQUIRED )
# 添加头文件
include_directories( SYSTEM ${OpenCV_INCLUDE_DIRS} /usr/local/opencv34/include )

ADD_EXECUTABLE( ppf ppf.cpp )
target_link_libraries( ppf ${OpenCV_LIBS} )

ADD_EXECUTABLE( ppf_normal_computation ppf_normal_computation.cpp )
target_link_libraries( ppf_normal_computation ${OpenCV_LIBS} )

ADD_EXECUTABLE( serialization serialization.cpp )
target_link_libraries( serialization ${OpenCV_LIBS} )
