#include <rviz_lasso_tool/rviz_lasso_tool.h>
#include <eigen_conversions/eigen_msg.h>
#include <pcl/filters/extract_indices.h>
#include <rviz_lasso_tool/ramer_douglas_peucker_simplification.h>
#include <Eigen/Dense>

static Eigen::Affine3d toEigen(const Ogre::Matrix4& m)
{
  Eigen::Affine3d r;
  r(0, 0) = m[0][0];
  r(0, 1) = m[0][1];
  r(0, 2) = m[0][2];
  r(0, 3) = m[0][3];

  r(1, 0) = m[1][0];
  r(1, 1) = m[1][1];
  r(1, 2) = m[1][2];
  r(1, 3) = m[1][3];

  r(2, 0) = m[2][0];
  r(2, 1) = m[2][1];
  r(2, 2) = m[2][2];
  r(2, 3) = m[2][3];

  r(3, 0) = m[3][0];
  r(3, 1) = m[3][1];
  r(3, 2) = m[3][2];
  r(3, 3) = m[3][3];

  return r;
}

static int pnpoly(int nvert, float *vertx, float *verty, float testx, float testy)
{
  int i, j, c = 0;
  for (i = 0, j = nvert-1; i < nvert; j = i++) {
    if ( ((verty[i]>testy) != (verty[j]>testy)) &&
         (testx < (vertx[j]-vertx[i]) * (testy-verty[i]) / (verty[j]-verty[i]) + vertx[i]) )
       c = !c;
  }
  return c;
}

static int pnpoly2(const std::vector<std::pair<float,float>>& verts, float testx, float testy)
{
  int i, j, c = 0;
  const int nvert = verts.size();
  const auto vertx = [&verts](int i) { return verts[i].first; };
  const auto verty = [&verts](int i) { return verts[i].second; };

  for (i = 0, j = nvert-1; i < nvert; j = i++) {
    if ( ((verty(i)>testy) != (verty(j)>testy)) &&
         (testx < (vertx(j)-vertx(i)) * (testy-verty(i)) / (verty(j)-verty(i)) + vertx(i)) )
       c = !c;
  }
  return c;
}

boost::shared_ptr<std::vector<int>> inside(const std::vector<std::pair<float,float>>& verts,
                        const pcl::PointCloud<pcl::PointXYZ>& cloud,
                        const Eigen::Affine3d& cam_pose_inv,
                        const Eigen::Affine3d& projection)
{
  auto indices = boost::make_shared<std::vector<int>>();

  for (std::size_t i = 0; i < cloud.size(); ++i)
  {
    const auto& pt = cloud[i];
    Eigen::Vector4d p_e (pt.x, pt.y, pt.z, 1);

//    Eigen::Vector4d tmp = cam_pose_inv * p_e;
//    ROS_INFO_STREAM("TMP: " << tmp.transpose());
    Eigen::Vector4d p = projection * cam_pose_inv * p_e;
//    ROS_INFO_STREAM("p_e: " << p_e.transpose());
//    ROS_INFO_STREAM("p: " << p.transpose());

    p /= p(2);
    if (pnpoly2(verts, p(0), p(1)))
    {
      indices->push_back(i);
    }
  }

  return indices;
}

rviz_lasso_tool::PointCloudTest::PointCloudTest(const std::string &path)
  : cloud_(new pcl::PointCloud<pcl::PointXYZ>)
{
  pcl::io::loadPCDFile(path, *cloud_);
  ros::NodeHandle nh;
  pub_ = nh.advertise<pcl::PointCloud<pcl::PointXYZ>>("inside", 1, true);
}

void rviz_lasso_tool::PointCloudTest::test(const std::vector<std::pair<float,float>>& verts, const geometry_msgs::PoseStamped &cam_pose, double focal_length, Ogre::Camera *cam)
{
  ROS_INFO("What what");

  auto proj1 = toEigen(cam->getProjectionMatrix());
  proj1.matrix()(2, 3) = 0.0;

  Eigen::Affine3d cam_pose_eigen;
  tf::poseMsgToEigen(cam_pose.pose, cam_pose_eigen);

  ROS_INFO_STREAM("Proj:\n" << proj1.matrix());
//  ROS_INFO_STREAM("Proj:\n" << proj1.matrix());

  auto indices = inside(rdp_simplification(verts, 0.01), *cloud_, cam_pose_eigen.inverse(), proj1);

  pcl::ExtractIndices<pcl::PointXYZ> extract;

//  pcl::IndicesPtr idx (new pcl::IndicesConstPtr)

  pcl::PointCloud<pcl::PointXYZ> inside_cloud;

  extract.setInputCloud(cloud_);
  extract.setIndices(indices);
  extract.setNegative (false);
  extract.filter(inside_cloud);

  inside_cloud.header.frame_id = "base_link";
  pub_.publish(inside_cloud);

}

