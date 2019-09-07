#include <sureclean_utils/planner_utils.h>

namespace sureclean {
void createTransform2D(Eigen::Matrix3d &transform, const double yaw,
                       const double dx, const double dy) {
  transform << std::cos(yaw), -std::sin(yaw), dx, //
      std::sin(yaw), std::cos(yaw), dy,           //
      0, 0, 1;                                    //
}
} // namespace sureclean