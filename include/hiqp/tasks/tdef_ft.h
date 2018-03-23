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

  int update(RobotStatePtr robot_state);

  int monitor();

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
  void FTcallback(const geometry_msgs::WrenchStamped& msg);
  int projectForces(std::shared_ptr<GeometricPoint>  point,
                    std::shared_ptr<GeometricPlane> plane,
                    const RobotStatePtr robot_state);

  std::shared_ptr<KDL::TreeFkSolverPos_recursive> fk_solver_pos_;
  std::shared_ptr<KDL::TreeJntToJacSolver> fk_solver_jac_;
  std::string sensor_frame_;
};

} // namespace tasks

} // namespace hiqp
