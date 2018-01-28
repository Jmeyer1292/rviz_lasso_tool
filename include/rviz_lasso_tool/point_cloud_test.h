#ifndef POINT_CLOUD_TEST_H
#define POINT_CLOUD_TEST_H

#include <pcl_ros/io/pcd_io.h>
#include <pcl_ros/point_cloud.h>
#include <pcl/point_cloud.h>

#include <OgreCamera.h>

namespace rviz_lasso_tool
{

class PointCloudTest
{
public:
  PointCloudTest(const std::string& path);

  void test(const std::vector<std::pair<float, float> > &verts, const geometry_msgs::PoseStamped& cam_pose,
            double focal_length, Ogre::Camera* cam);

private:
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_;
  ros::Publisher pub_;
};

}

#endif // POINT_CLOUD_TEST_H
