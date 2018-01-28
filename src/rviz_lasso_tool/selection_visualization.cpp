#include "rviz_lasso_tool/selection_visualization.h"
#include "rviz_lasso_tool/ramer_douglas_peucker_simplification.h"

#include <ros/console.h>

rviz_lasso_tool::SelectionVisualization::SelectionVisualization(const Ogre::String &name)
  : Ogre::ManualObject(name)
{
  setRenderQueueGroup(Ogre::RENDER_QUEUE_OVERLAY);
  setUseIdentityProjection(true);
  setUseIdentityView(true);
  setQueryFlags(0);
  setBoundingBox(Ogre::AxisAlignedBox::BOX_INFINITE);
}

rviz_lasso_tool::SelectionVisualization::~SelectionVisualization()
{

}

void rviz_lasso_tool::SelectionVisualization::setCorners(float left, float top, float right, float bottom)
{
  clear();
  begin("Examples/KnotTexture", Ogre::RenderOperation::OT_LINE_STRIP);
  position(left,top,-1);
  position(right, top, -1);
  position(right, bottom, -1);
  position(left, bottom, -1);
  position(left, top, -1);
  end();

//  setBoundingBox(Ogre::AxisAlignedBox::BOX_INFINITE);
}

void rviz_lasso_tool::SelectionVisualization::setPolygon(const std::vector<std::pair<float, float>>& coords)
{
  ROS_INFO_STREAM("INPUT COORDS: " << coords.size());
  auto clone = rdp_simplification(coords, 0.01);

  ROS_INFO_STREAM("After RDP: " << clone.size());

  clear();
  begin("Examples/KnotTexture", Ogre::RenderOperation::OT_LINE_STRIP);

  for (const auto& pos : clone)
  {
    position(pos.first, pos.second, -1);
  }

  if (!clone.empty()) // Complete the lasso loop
  {
    position(clone.front().first, clone.front().second, -1);
  }

  end();
}
