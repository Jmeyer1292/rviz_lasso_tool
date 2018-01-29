#ifndef RVIZ_LASSO_TOOL_H
#define RVIZ_LASSO_TOOL_H

#include <rviz/tool.h>
#include <rviz_lasso_tool/selection_visualization.h>
#include <ros/publisher.h>

#include <geometry_msgs/PoseStamped.h>
#include <Eigen/Dense>

namespace rviz_lasso_tool
{

class RvizLassoTool : public rviz::Tool
{
  Q_OBJECT
public:
  RvizLassoTool();

  virtual void onInitialize() override;

  virtual void activate() override;
  virtual void deactivate() override;

  virtual int processMouseEvent(rviz::ViewportMouseEvent& event) override;

  void publishSelection(const rviz::ViewportMouseEvent& event);

private:
  SelectionVisualization* vis_;
  std::vector<std::pair<float,float>> current_selection_;
  int last_x_;
  int last_y_;
  ros::Publisher pub_;
};

}

#endif // RVIZ_LASSO_TOOL
