#include "rviz_lasso_tool/rviz_lasso_tool.h"
#include <rviz/viewport_mouse_event.h>
#include <ros/console.h>
#include <OGRE/OgreSceneManager.h>

#include <pluginlib/class_list_macros.h>

#include <rviz/display_context.h>
#include <ros/node_handle.h>

static float toRelative(const int sample, const int axis_length)
{
  return (static_cast<float>(sample) / axis_length) * 2 - 1.0;
}

void rviz_lasso_tool::RvizLassoTool::onInitialize() {
  ROS_INFO("On init");
  vis_ = nullptr;

//  ros::NodeHandle nh;
//  pub_ = nh.advertise<std_msgs::Lasso>
}

void rviz_lasso_tool::RvizLassoTool::activate() {
  ROS_INFO("activate");
  vis_ = new SelectionVisualization("lasso_vis");
  scene_manager_->getRootSceneNode()->createChildSceneNode()->attachObject(vis_);
}

void rviz_lasso_tool::RvizLassoTool::deactivate() {
  ROS_INFO("De-activate");
  delete vis_;
}

int rviz_lasso_tool::RvizLassoTool::processMouseEvent(rviz::ViewportMouseEvent &event)
{
  const auto width = event.viewport->getActualWidth();
  const auto height = event.viewport->getActualHeight();

  if (event.leftDown()) // Start new selection event
  {
    ROS_INFO("Start");
    start_rel_x_ = toRelative(event.x, width);
    start_rel_y_ = -toRelative(event.y, height);

    last_x_ = event.x;
    last_y_ = event.y;
  }
  else if (event.leftUp()) // End this selection event
  {
    ROS_INFO("Done");
    publishCurrent();

    Ogre::Camera* cam = event.viewport->getCamera();
    const auto focal_length = cam->getFocalLength();
    const auto cam_pos = cam->getPosition();
    const auto cam_orient = cam->getOrientation();



    current_selection_.clear();
  }
  else if (event.left()) // Continue this selection event
  {
    ROS_INFO("Continue");

    if (last_x_ == event.x && last_y_ == event.y)
    {
      return 0;
    }

    last_x_ = event.x;
    last_y_ = event.y;

    ROS_INFO("Clicked %d, %d in screen that is %d by %d", event.x, event.y, width, height);

    const auto relative_x = toRelative(event.x, width);
    const auto relative_y = -toRelative(event.y, height);

    ROS_INFO("Clicked rel %f, %f", relative_x, relative_y);
    current_selection_.emplace_back(relative_x, relative_y);

    vis_->setPolygon(current_selection_);
  }
  return 0;
}

void rviz_lasso_tool::RvizLassoTool::publishCurrent()
{
}

PLUGINLIB_EXPORT_CLASS(rviz_lasso_tool::RvizLassoTool, rviz::Tool)
