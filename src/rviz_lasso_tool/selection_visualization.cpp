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
}

rviz_lasso_tool::SelectionVisualization::~SelectionVisualization()
{}

void rviz_lasso_tool::SelectionVisualization::setPolygon(const std::vector<std::pair<float, float>>& coords)
{
  auto clone = rdp_simplification(coords, 0.01);
  clear();
  begin("BaseWhiteNoLighting", Ogre::RenderOperation::OT_LINE_STRIP);

  for (const auto& pos : clone) position(pos.first, pos.second, -1);

  if (!clone.empty()) // Complete the lasso loop
    position(clone.front().first, clone.front().second, -1);

  end();
  setBoundingBox(Ogre::AxisAlignedBox::BOX_INFINITE);
}
