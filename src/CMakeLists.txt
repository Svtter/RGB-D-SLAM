# pcl support
find_package( PCL REQUIRED COMPONENTS common io )
find_package( OpenCV REQUIRED )

# add headers
ADD_DEFINITIONS( ${PCL_DEFINATIONS} )
INCLUDE_DIRECTORIES( ${PCL_INCLUDE_DIRS} )
LINK_LIBRARIES( ${PCL_LIBRARY_DIRS} )

ADD_EXECUTABLE( generate_pointcloud generatePointCloud.cpp )
TARGET_LINK_LIBRARIES( generate_pointcloud ${OpenCV_LIBS}
    ${PCL_LIBRARIES} )
