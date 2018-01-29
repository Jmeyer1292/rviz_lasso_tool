#ifndef SELECTION_VISUALIZATION_H
#define SELECTION_VISUALIZATION_H

#include <OGRE/OgreManualObject.h>

namespace rviz_lasso_tool
{

class SelectionVisualization : public Ogre::ManualObject
{
public:
  SelectionVisualization(const Ogre::String& name);
  virtual ~SelectionVisualization();

  void setPolygon(const std::vector<std::pair<float,float>>& coords);
};

}

#endif // SELECTION_VISUALIZATION_H
