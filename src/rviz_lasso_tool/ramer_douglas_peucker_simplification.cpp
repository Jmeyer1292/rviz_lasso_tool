#include "rviz_lasso_tool/ramer_douglas_peucker_simplification.h"
#include <cmath>
#include <algorithm>

float perpendicular_distance(const rviz_lasso_tool::Point &pt, const rviz_lasso_tool::Point &line_start,
                              const rviz_lasso_tool::Point &line_end)
{
  float dx = line_end.first - line_start.first;
  float dy = line_end.second - line_start.second;

  //Normalise
  float mag = std::pow(std::pow(dx,2.0)+std::pow(dy,2.0),0.5);
  if(mag > 0.0)
  {
    dx /= mag; dy /= mag;
  }

  float pvx = pt.first - line_start.first;
  float pvy = pt.second - line_start.second;

  //Get dot product (project pv onto normalized direction)
  float pvdot = dx * pvx + dy * pvy;

  //Scale line direction vector
  float dsx = pvdot * dx;
  float dsy = pvdot * dy;

  //Subtract this from pv
  float ax = pvx - dsx;
  float ay = pvy - dsy;

  return std::pow(std::pow(ax,2.0)+std::pow(ay,2.0),0.5);
}

std::vector<rviz_lasso_tool::Point> rviz_lasso_tool::rdp_simplification(const std::vector<Point>& points,
                                                                        const float eps)
{
  if (points.size() <= 2)
    return points;

  const auto& start_pt = points.front();
  const auto& end_pt = points.back();

  std::size_t max_index = 1;
  float max_dist = 0.0;
  for (std::size_t i = 1; i < points.size() - 1; ++i)
  {
    const auto dist = perpendicular_distance(points[i], start_pt, end_pt);
    if (dist > max_dist)
    {
      max_dist = dist;
      max_index = i;
    }
  }

  if (max_dist > eps)
  {
    // Recursively sub-divide
    std::vector<Point> front_half_points (points.begin(), points.begin() + max_index + 1);
    std::vector<Point> back_half_points (points.begin() + max_index, points.end());

    auto front_half_result = rdp_simplification(front_half_points, eps);
    auto back_half_result = rdp_simplification(back_half_points, eps);

    std::vector<Point> result = std::move(front_half_result);
    result.insert(result.end(), back_half_result.begin() + 1, back_half_result.end());
    return result;
  }
  else
  {
    std::vector<Point> result = {start_pt, end_pt};
    return result;
  }
}
