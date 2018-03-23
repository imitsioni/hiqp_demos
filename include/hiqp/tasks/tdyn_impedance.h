// Adjusting the tdyn_linear_impedance code to be integrated in the FACT Framework
// but as a plugin (e.g. tdyn_pd_controller)
#ifndef HIQP_TDYN_IMP_H
#define HIQP_TDYN_IMP_H

#include <geometry_msgs/WrenchStamped.h>
#include <hiqp/robot_state.h>
#include <hiqp/tasks/tdyn_pd_controller.h>
// #include <pluginlib/class_loader.h>
// #include <ros/ros.h>


namespace hiqp{
  namespace tasks{

/*! \brief A linear impedance controller
 *
 * The code is based on the typical second order system:
 * *   M * x_ddot + K_d * x_dot + K_p * x = F_ext
 * that corresponds to the response:
 * *   s^2 + 2*zeta*w_n + w_n^2 = 0
 * where zeta is the damping ratio, w_n the natural frequency of the system an are
 * related through the settling time as follows:
 * *    t_s = 4 / (zeta * w_n)
 */

  class TDynImpedance : public TDynPDController {
  public:
    inline TDynImpedance() : TDynPDController() {}

    TDynImpedance(std::shared_ptr<GeometricPrimitiveMap> geom_prim_map,
                         std::shared_ptr<Visualizer> visualizer);

    ~TDynImpedance() noexcept {}

    int init(const std::vector<std::string> &parameters,
             RobotStatePtr robot_state,
             const Eigen::VectorXd &e_initial,
             const Eigen::VectorXd &e_dot_initial,
             const Eigen::VectorXd &e_final,
             const Eigen::VectorXd &e_dot_final);

    int update(const RobotStatePtr robot_state,
               const std::shared_ptr<TaskDefinition> def);

    int monitor();





    protected:
    // double k_p_ ; // stiffness
    // double k_d_ ;// damping
    // //double B_inv_;  // desired inertia matrix, added according to the initial code
    Eigen::MatrixXd B_inv_ ; //pinv of desired inertia matrix



    private:

    TDynImpedance(const TDynImpedance& other) = delete;
    TDynImpedance(TDynImpedance&& other) = delete;
    TDynImpedance& operator=(const TDynImpedance& other) = delete;
    TDynImpedance& operator=(TDynImpedance&& other) noexcept = delete;

      };
    }// ns tasks
    }// ns hiqp

#endif
