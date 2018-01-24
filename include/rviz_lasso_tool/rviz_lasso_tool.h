#ifndef RVIZ_LASSO_TOOL_H
#define RVIZ_LASSO_TOOL_H

#include <rviz/tool.h>
#include <rviz_lasso_tool/selection_visualization.h>
#include <ros/publisher.h>

namespace rviz_lasso_tool
{

class RvizLassoTool : public rviz::Tool
{
  Q_OBJECT
public:

  virtual void onInitialize();

  virtual void activate();
  virtual void deactivate();

  virtual int processMouseEvent( rviz::ViewportMouseEvent& event );

  void publishCurrent();

private:
  std::vector<std::pair<float,float>> current_selection_;
  SelectionVisualization* vis_;

  float start_rel_x_;
  float start_rel_y_;
  int last_x_;
  int last_y_;

  ros::Publisher pub_;

};

}

#endif // RVIZ_LASSO_TOOL
