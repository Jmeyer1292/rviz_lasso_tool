#include "rviz_lasso_tool/rviz_lasso_tool.h"
#include <rviz/viewport_mouse_event.h>
#include <ros/console.h>
#include <OgreSceneManager.h>

#include <pluginlib/class_list_macros.h>

#include <rviz/display_context.h>
#include <ros/node_handle.h>

#include <rviz_lasso_tool/UserSelection.h>

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

static float toRelative(const int sample, const int axis_length)
{
  return (static_cast<float>(sample) / axis_length) * 2 - 1.0;
}

rviz_lasso_tool::RvizLassoTool::RvizLassoTool()
  : vis_{nullptr}
{}

void rviz_lasso_tool::RvizLassoTool::onInitialize()
{
  this->shortcut_key_ = 'l';

  ros::NodeHandle nh;
  pub_ = nh.advertise<rviz_lasso_tool::UserSelection>("user_selection", 1, false);
}

void rviz_lasso_tool::RvizLassoTool::activate()
{
  vis_ = new SelectionVisualization("lasso_vis");
  scene_manager_->getRootSceneNode()->createChildSceneNode()->attachObject(vis_);
}

void rviz_lasso_tool::RvizLassoTool::deactivate()
{
  delete vis_;
}

int rviz_lasso_tool::RvizLassoTool::processMouseEvent(rviz::ViewportMouseEvent &event)
{
  const auto width = event.viewport->getActualWidth();
  const auto height = event.viewport->getActualHeight();

  if (event.leftDown()) // Start new selection event
  {
    last_x_ = event.x;
    last_y_ = event.y;

    const auto relative_x = toRelative(event.x, width);
    const auto relative_y = -toRelative(event.y, height);
    current_selection_.emplace_back(relative_x, relative_y);
  }
  else if (event.left()) // Continue this selection event
  {
    if (last_x_ == event.x && last_y_ == event.y)
    {
      return rviz::Tool::Render;
    }
    last_x_ = event.x;
    last_y_ = event.y;

    const auto relative_x = toRelative(event.x, width);
    const auto relative_y = -toRelative(event.y, height);
    current_selection_.emplace_back(relative_x, relative_y);
    // Visualize
    vis_->setPolygon(current_selection_);
  }
  else if (event.leftUp()) // End this selection event
  {
    publishSelection(event);
    current_selection_.clear();
    return rviz::Tool::Finished;
  }

  return rviz::Tool::Render;
}

static std::vector<geometry_msgs::Point32> toMsg(const std::vector<std::pair<float,float>>& verts)
{
  std::vector<geometry_msgs::Point32> pts;
  pts.reserve(verts.size());
  for (const auto& vert : verts)
  {
    geometry_msgs::Point32 pt;
    pt.x = vert.first;
    pt.y = vert.second;
    pt.z = 0.0f;
    pts.push_back(pt);
  }
  return pts;
}

static boost::array<float, 16> toArrayMsg(const Eigen::Affine3d& e)
{
  boost::array<float, 16> m;
  m[0] = e(0, 0);
  m[1] = e(0, 1);
  m[2] = e(0, 2);
  m[3] = e(0, 3);

  m[4] = e(1, 0);
  m[5] = e(1, 1);
  m[6] = e(1, 2);
  m[7] = e(1, 3);

  m[8] = e(2, 0);
  m[9] = e(2, 1);
  m[10] = e(2, 2);
  m[11] = e(2, 3);

  m[12] = e(3, 0);
  m[13] = e(3, 1);
  m[14] = e(3, 2);
  m[15] = e(3, 3);
  return m;
}

static Eigen::Affine3d fromArrayMsg(const boost::array<float, 16>& m)
{
  Eigen::Affine3d e;
  e(0, 0) = m[0];
  e(0, 1) = m[1];
  e(0, 2) = m[2];
  e(0, 3) = m[3];
  e(1, 0) = m[4];
  e(1, 1) = m[5];
  e(1, 2) = m[6];
  e(1, 3) = m[7];
  e(2, 0) = m[8];
  e(2, 1) = m[9];
  e(2, 2) = m[10];
  e(2, 3) = m[11];
  e(3, 0) = m[12];
  e(3, 1) = m[13];
  e(3, 2) = m[14];
  e(3, 3) = m[15];
  return e;
}

void rviz_lasso_tool::RvizLassoTool::publishSelection(const rviz::ViewportMouseEvent &event)
{
  Ogre::Camera* cam = event.viewport->getCamera();
  const auto cam_pos = cam->getPosition();
  const auto cam_orient = cam->getOrientation();

  // Fetch this data
  const auto fixed_frame = this->context_->getFixedFrame().toStdString();

  geometry_msgs::PoseStamped pose_stamped;
  pose_stamped.header.frame_id = fixed_frame;

  pose_stamped.pose.position.x = cam_pos.x;
  pose_stamped.pose.position.y = cam_pos.y;
  pose_stamped.pose.position.z = cam_pos.z;

  pose_stamped.pose.orientation.x = cam_orient.x;
  pose_stamped.pose.orientation.y = cam_orient.y;
  pose_stamped.pose.orientation.z = cam_orient.z;
  pose_stamped.pose.orientation.w = cam_orient.w;

  auto proj = toEigen(cam->getProjectionMatrix());
  proj(2, 3) = 0.0;

  rviz_lasso_tool::UserSelection selection;
  selection.camera_pose = pose_stamped;
  selection.vertices = toMsg(current_selection_);
  selection.proj_matrix = toArrayMsg(proj);
  selection.shift = static_cast<bool>(event.modifiers & Qt::ShiftModifier);
  selection.control = static_cast<bool>(event.modifiers & Qt::ControlModifier);
  selection.alt = static_cast<bool>(event.modifiers & Qt::AltModifier);

  pub_.publish(selection);
}

PLUGINLIB_EXPORT_CLASS(rviz_lasso_tool::RvizLassoTool, rviz::Tool)
