// This program is free software: you can redistribute it and/or modify
// it under the terms of the GNU General Public License as published by
// the Free Software Foundation, either version 3 of the License, or
// (at your option) any later version.
//
// This program is distributed in the hope that it will be useful,
// but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
// GNU General Public License for more details.
//
// You should have received a copy of the GNU General Public License
// along with this program.  If not, see <http://www.gnu.org/licenses/>.

#ifndef HIQP_TDYN_PD_CONTROLLER_H
#define HIQP_TDYN_PD_CONTROLLER_H

#include "std_msgs/String.h"
#include <hiqp/robot_state.h>
#include <hiqp/task_dynamics.h>
#include <pluginlib/class_loader.h>
#include <ros/ros.h>

namespace hiqp {
namespace tasks {

/*! \brief A simple PD-controller
 *  \author Robert Krug */
class TDynPDController : public TaskDynamics {
public:
  inline TDynPDController() : TaskDynamics() {}

  TDynPDController(std::shared_ptr<GeometricPrimitiveMap> geom_prim_map,
                  std::shared_ptr<Visualizer> visualizer);

  ~TDynPDController() noexcept {}

  int init(const std::vector<std::string> &parameters,
           RobotStatePtr robot_state, const Eigen::VectorXd &e_initial,
           const Eigen::VectorXd &e_dot_initial, const Eigen::VectorXd &e_final,
           const Eigen::VectorXd &e_dot_final);

  int update(const RobotStatePtr robot_state,
             const std::shared_ptr<TaskDefinition> def);

  int monitor();

protected:
  double k_p_; ///< Controller proportional gain
  double k_d_; ///< Controller derivative gain  

private:
  TDynPDController(const TDynPDController &other) = delete;
  TDynPDController(TDynPDController &&other) = delete;
  TDynPDController &operator=(const TDynPDController &other) = delete;
  TDynPDController &operator=(TDynPDController &&other) noexcept = delete;

  /* ros::Publisher starting_pub_; */
  /* std_msgs::String msg_; */
  /* ros::ServiceClient client_NN_; */
  /* ros::Time t; */
  /* ros::NodeHandle nh_; */
  /* double lambda_; */
};

} // namespace tasks

} // namespace hiqp

#endif // include guard
