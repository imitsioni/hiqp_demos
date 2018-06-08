// Hardcoded Force projection.
// Only point to plane projection is supported
// 21-03: all the geometric info are hardcoded. We always assume it's the z-plane
//        we're interested in and the forces are being propagated through a ros wrench_topic


// #ifndef HIQP_TDEF_FT_H
// #define HIQP_TDEF_FT_H
#pragma once

#include <string>
#include <vector>

#include <hiqp/task_definition.h>
// #include <hiqp/robot_state.h>
#include <geometry_msgs/WrenchStamped.h>
#include <kdl/treefksolverpos_recursive.hpp>
#include <kdl/treejnttojacsolver.hpp>
#include <pluginlib/class_loader.h>
#include <ros/ros.h>

namespace hiqp {
namespace tasks {


class TDefFT : public TaskDefinition {
public:
  inline TDefFT() : TaskDefinition() {}

  TDefFT(std::shared_ptr<GeometricPrimitiveMap> geom_prim_map,
            std::shared_ptr<Visualizer> visualizer);
  ~TDefFT() noexcept;

  int init(const std::vector<std::string> &parameters,
           RobotStatePtr robot_state);
  /* This function :
   *                 * Parses the arguments from the task definition (sensor frame included)
   *                 * Subscribes to the force reading wrench_topic
   *                 * Initializes explicitly the main class (TDefGeometricProjection)
   *                           /TODO remove the extra code, initialize it like a normal human being
   *                 * Sets up the fk and jacobian solvers
   */
  int update(RobotStatePtr robot_state);
 /* This function : * Gets cartesian poses from the primitives in task definition
  *                 * Gets the jacobians for the primitives in task definition
  *                 * Gets the derivatives of the jacobians for the primitives in task definition
  *                 * Updates explicitly the main class (TDefGeometricProjection)
  *                           /TODO remove the extra code, update it like a normal human being
  *                 * Transform primitives to world frame
  *                 * Projects forces
  *                           /TODO add the knife_frame to tf
  */
  int monitor(); // Does nothing for the time being


protected:
   std::shared_ptr<GeometricPoint> primitive_a_;
   std::shared_ptr<GeometricPlane> primitive_b_;

   KDL::Frame pose_a_;
   KDL::Jacobian jacobian_a_; ///< tree jacobian w.r.t. the center of the frame
                             ///TDefGeometricProjection::pose_a_
   KDL::Jacobian jacobian_dot_a_;

   KDL::Frame pose_b_;
   KDL::Jacobian jacobian_b_; ///< tree jacobian w.r.t. the center of the frame
                             ///TDefGeometricProjection::pose_b_
   KDL::Jacobian jacobian_dot_b_;

private:
  TDefFT(const TDefFT &other) = delete;
  TDefFT(TDefFT &&other) = delete;
  TDefFT &operator=(const TDefFT &other) = delete;
  TDefFT &operator=(TDefFT &&other) noexcept = delete;

  // Eigen::Matrix<double, 3, 1> f_;
  Eigen::Matrix<double, 6, 1> Wrench_fts_frame;
  ros::NodeHandle nh_;
  ros::Subscriber wrench_sub;
  void FTcallback(const geometry_msgs::WrenchStamped::ConstPtr& msg);
  // The callback function for the wrench wrench_topic

  int projectForces(std::shared_ptr<GeometricPoint>  point,
                    std::shared_ptr<GeometricPlane> plane,
                    const RobotStatePtr robot_state);
  /* This function : * Expresses the received _FORCE_ in the world frame
   *                 * Expresses the plane normal (primitive b) in the world frame
   *                 * Sets a hardcoded plane_projection matrix by n*n^T
   *                           /TODO do it through the base class method
   *                 * Sets f_ as the projected force for the TDyn
   */

  std::shared_ptr<KDL::TreeFkSolverPos_recursive> fk_solver_pos_;
  std::shared_ptr<KDL::TreeJntToJacSolver> fk_solver_jac_;
  std::string sensor_frame_;

};

} // namespace tasks

} // namespace hiqp
