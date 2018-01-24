#ifndef RAMER_DOUGLAS_PEUCKER_SIMPLIFICATION_H
#define RAMER_DOUGLAS_PEUCKER_SIMPLIFICATION_H

#include <vector>

namespace rviz_lasso_tool
{

using Point = std::pair<float, float>;

std::vector<Point> rdp_simplification(const std::vector<Point>& points, const float eps);

}

#endif // RAMER_DOUGLAS_PEUCKER_SIMPLIFICATION_H
