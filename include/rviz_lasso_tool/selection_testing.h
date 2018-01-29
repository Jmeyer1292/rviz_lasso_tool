#ifndef RVIZ_LASSO_SELECTION_TESTING_H
#define RVIZ_LASSO_SELECTION_TESTING_H

#include <vector>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <boost/array.hpp>
#include <geometry_msgs/Point32.h>

namespace rviz_lasso_tool
{

inline int pnpoly(int nvert, float *vertx, float *verty, float testx, float testy)
{
  int i, j, c = 0;
  for (i = 0, j = nvert-1; i < nvert; j = i++) {
    if ( ((verty[i]>testy) != (verty[j]>testy)) &&
         (testx < (vertx[j]-vertx[i]) * (testy-verty[i]) / (verty[j]-verty[i]) + vertx[i]) )
       c = !c;
  }
  return c;
}

inline int pnpoly2(const std::vector<std::pair<float,float>>& verts, float testx, float testy)
{
  int i, j, c = 0;
  const int nvert = verts.size();
  const auto vertx = [&verts](int i) { return verts[i].first; };
  const auto verty = [&verts](int i) { return verts[i].second; };

  for (i = 0, j = nvert-1; i < nvert; j = i++) {
    if ( ((verty(i)>testy) != (verty(j)>testy)) &&
         (testx < (vertx(j)-vertx(i)) * (testy-verty(i)) / (verty(j)-verty(i)) + vertx(i)) )
       c = !c;
  }
  return c;
}

template<class PointT>
inline boost::shared_ptr<std::vector<int>> inside(const std::vector<std::pair<float,float>>& verts,
                        const pcl::PointCloud<PointT>& cloud,
                        const Eigen::Affine3d& cam_pose_inv,
                        const Eigen::Affine3d& projection)
{
  auto indices = boost::make_shared<std::vector<int>>();

  for (std::size_t i = 0; i < cloud.size(); ++i)
  {
    const auto& pt = cloud[i];
    Eigen::Vector4d p_e (pt.x, pt.y, pt.z, 1);
    Eigen::Vector4d p = projection * cam_pose_inv * p_e;
    p /= p(2);
    if (pnpoly2(verts, p(0), p(1)))
    {
      indices->push_back(i);
    }
  }

  return indices;
}

inline std::vector<geometry_msgs::Point32> toMsg(const std::vector<std::pair<float,float>>& verts)
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

inline std::vector<std::pair<float,float>> fromMsg(const std::vector<geometry_msgs::Point32>& points)
{
  std::vector<std::pair<float,float>> verts;
  verts.reserve(points.size());
  for (const auto& p : points)
  {
    verts.emplace_back(p.x, p.y);
  }
  return verts;
}

inline boost::array<float, 16> toArrayMsg(const Eigen::Affine3d& e)
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

inline Eigen::Affine3d fromArrayMsg(const boost::array<float, 16>& m)
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

}

#endif // RVIZ_LASSO_SELECTION_TESTING_H
