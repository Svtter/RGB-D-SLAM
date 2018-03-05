
ADD_EXECUTABLE( main main.cpp )

target_link_libraries( main realsense2 ${OpenCV_LIBS} )
# target_link_libraries( main  ${OpenCV_LIBS} ))