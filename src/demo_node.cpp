#include <ros/ros.h>

#include <eigen_conversions/eigen_msg.h>

#include <pcl/io/pcd_io.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/filters/extract_indices.h>
#include <pcl_ros/point_cloud.h>

#include <rviz_lasso_tool/ramer_douglas_peucker_simplification.h>
#include <rviz_lasso_tool/selection_testing.h>
#include <rviz_lasso_tool/UserSelection.h>

class UserSelectionManager
{
  const static uint8_t ACTIVE_R = 255;
  const static uint8_t ACTIVE_G = 0;
  const static uint8_t ACTIVE_B = 0;

  const static uint8_t INACTIVE_R = 255;
  const static uint8_t INACTIVE_G = 255;
  const static uint8_t INACTIVE_B = 255;

public:
  UserSelectionManager(ros::NodeHandle& nh, const std::string& pcd_path)
    : cloud_(new pcl::PointCloud<pcl::PointXYZRGB>)
  {
    selection_sub_ = nh.subscribe<rviz_lasso_tool::UserSelection>("user_selection", 1,
                                                                  &UserSelectionManager::onUserSelection, this);

    cloud_pub_ = nh.advertise<pcl::PointCloud<pcl::PointXYZRGB>>("color_cloud", 1, true);

    if (pcl::io::loadPCDFile(pcd_path, *cloud_) == -1)
    {
      throw std::runtime_error("Unable to load pcd file: " + pcd_path);
    }
    cloud_->header.frame_id = "base_link";
    active_map_.resize(cloud_->size(), false);

    colorAndPublish();
  }

  void onUserSelection(const rviz_lasso_tool::UserSelectionConstPtr& msg)
  {
    // Step 1: Convert selection to native type
    auto verts = rviz_lasso_tool::fromMsg(msg->vertices);
    verts = rviz_lasso_tool::rdp_simplification(verts, 0.01);
    auto projection = rviz_lasso_tool::fromArrayMsg(msg->proj_matrix);

    Eigen::Affine3d inv_cam_pose;
    tf::poseMsgToEigen(msg->camera_pose.pose, inv_cam_pose);
    inv_cam_pose = inv_cam_pose.inverse();

    // Step 2: Determinate what points are in the selection area
    boost::shared_ptr<std::vector<int>> in_points = rviz_lasso_tool::inside(verts, *cloud_, inv_cam_pose, projection);

    // Based on the key modifiers, choose an action
    if (msg->control) // add to existing selection
    {
      ROS_INFO("Control");
      mark(*in_points, true);
    }
    else if (msg->shift) // remove selected points
    {
      ROS_INFO("Shift");
      mark(*in_points, false);
    }
    else // new selection
    {
      ROS_INFO("New");
      reset();
      mark(*in_points, true);
    }

    colorAndPublish();
  }

  void reset()
  {
    std::fill(active_map_.begin(), active_map_.end(), false);
  }

  void mark(const std::vector<int>& indices, bool active)
  {
    for (auto i : indices)
    {
      active_map_[i] = active;
    }
  }

  void colorAndPublish()
  {
    for (std::size_t i = 0; i < active_map_.size(); ++i)
    {
      if (active_map_[i])
      {
        (*cloud_)[i].r = ACTIVE_R;
        (*cloud_)[i].g = ACTIVE_G;
        (*cloud_)[i].b = ACTIVE_B;
      }
      else
      {
        (*cloud_)[i].r = INACTIVE_R;
        (*cloud_)[i].g = INACTIVE_G;
        (*cloud_)[i].b = INACTIVE_B;
      }
    }
    ROS_INFO_STREAM("publish");
    cloud_pub_.publish(*cloud_);
  }

private:
  ros::Subscriber selection_sub_;
  ros::Publisher cloud_pub_;

  pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_;
  std::vector<bool> active_map_;
};

int main(int argc, char** argv)
{
  ros::init(argc, argv, "selection_demo_node");
  ros::NodeHandle nh, pnh ("~");

  // Load parameters
  std::string pcd_path;
  if (!pnh.getParam("pcd_path", pcd_path))
  {
    ROS_FATAL("Must set 'pcd_path' param");
    return 1;
  }

  UserSelectionManager manager (nh, pcd_path);

  ros::spin();
  return 0;
}
