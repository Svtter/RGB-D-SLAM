#include <iostream>
#include <string>

using namespace std;

// OPENCV
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>

// PCL
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>

// define pointcloud type
typedef pcl::PointXYZRGBA PointT;
typedef pcl::PointCloud<PointT> PointCloud;

// camera parameters
const double camera_factor = 1000;
const double camera_cx = 325.5;
const double camera_cy = 253.5;
const double camera_fx = 518.0;
const double camera_fy = 519.0;

// 主函数
int main( int argc, char ** argv )
{
  // read ./data/rgb.png && ./data/depth.png, trans to pointcloud

  // Pic Mat
  cv::Mat rgb, depth;

  // use cv::imread() to read pic
  // API: http://docs.opencv.org/modules/highgui/doc/reading_and_writing_images_and_video.html?highlight=imread#cv2.imread
  rgb = cv::imread( "./data/rgb.png" );

  // rgb is 8UC3
  // depth is 16UC1
  depth = cv::imread( "./data/depth.png", -1 );

  // Point Var
  // smart pointer
  PointCloud::Ptr cloud( new PointCloud );

  // walk through depth
  for (int m = 0; m < depth.rows; m++)
    for(int n = 0; n < depth.cols; n++)
      {
        // get depth(m, n)
        ushort d = depth.ptr<ushort>(m)[n];

        // d might have no value
        if (d == 0)
          continue;

        // d exists, add a point to cloud.
        PointT p;

        // cal this point location
        p.z = double(d) / camera_factor;
        p.x = (n - camera_cx) * p.z / camera_fx;
        p.y = (m - camera_cy) * p.z / camera_fy;

        // get color from rgb
        // rgb 3-channel bgr
        p.b = rgb.ptr<uchar>(m)[n*3];
        p.g = rgb.ptr<uchar>(m)[n*3 + 1];
        p.r = rgb.ptr<uchar>(m)[n*3 + 2];

        // put p into pointcloud

        cloud->points.push_back( p );
      }

  // set & save point cloud

  cloud->height = 1;
  cloud->width = cloud->points.size();
  cout << "point cloud size" << cloud->points.size() << endl;
  cloud->is_dense = false;
  pcl::io::savePCDFile( "./pointcloud.pcd", *cloud );

  // clear data & exit
  cloud->points.clear();
  cout << "Point cloud saved." << endl;
  return 0;

}
