#include <limits>
#include <random>
#include <hiqp/utilities.h>
#include <hiqp/tasks/tdyn_impedance.h>
#include <pluginlib/class_list_macros.h>
#include <ros/ros.h>


namespace hiqp{
  namespace tasks{

    TDynImpedance::TDynImpedance(
      std::shared_ptr<GeometricPrimitiveMap> geom_prim_map,
      std::shared_ptr<Visualizer> visualizer)
      : TDynPDController(geom_prim_map, visualizer) {}

    int TDynImpedance::init(const std::vector<std::string> &parameters,
             RobotStatePtr robot_state,
             const Eigen::VectorXd &e_initial,
             const Eigen::VectorXd &e_dot_initial,
             const Eigen::VectorXd &e_final,
             const Eigen::VectorXd &e_dot_final){

       int dim = e_initial.rows();   ///< number of controlled dimensions
       int size = parameters.size(); ///< number of given parameters

       // ensure consistency
       assert((e_dot_initial.rows() == dim) && (e_final.rows() == dim) &&
              (e_dot_final.rows() == dim));


      /*! \brief It is assumed that the two gain matrices Kp & Kd in R^{m x m}
       * are given subsequently in row-major format in the parameter string,
       * i. e., Kp_ = parameters[1] - parameters[m^2],
       * and Kd_ = parameters[m^2+1] - parameters[2*m^2],
       * where m is the task space dimension
       * \author Robert
       */


      /*  ensure that valid parameters are given
       *  [0] : Controller name,  [1] : K_p , [2] : K_d , [3]: B
       */



       /*  ======================   DESTROY
       */
      wrench_sub = nh_.subscribe(wrench_topic, 10, &TDynImpedance::FTcallback);

      if ((size != 3 * dim * dim + 1) && (size != 4)) {
        printHiqpWarning("TDynImpedance for a " + std::to_string(dim) +
                         "-dimensional task requires either " +
                         std::to_string(3 * dim * dim + 1) +
                         " or 4 parameters, got " + std::to_string(size) +
                         "! Initialization failed!");
        return -1;
      }


      // read the gains and the desired inertia matrix
       k_p_ = std::stod(parameters.at(1));
       k_d_ = std::stod(parameters.at(2));

      Eigen::MatrixXd B;
      std::vector<std::string> tdyn_pd_parameters;


      if (size == 4) {
        B = Eigen::MatrixXd::Identity(dim, dim) * std::stod(parameters.at(3));
        // extract the parameters for the base class
        for (unsigned int i = 0; i < size - 1; i++)
          tdyn_pd_parameters.push_back(parameters.at(i));
      }

      else { // read the full B matrix
        double *B_data = new double[dim * dim];
        for (int i = 0; i < dim * dim; i++)
          B_data[i] = std::stod(parameters.at(i + 1 + 2 * dim * dim));

        B = (Eigen::Map<Eigen::MatrixXd>(B_data, dim, dim)).transpose();
        delete B_data;
        // extract the parameters for the base class
        for (unsigned int i = 0; i < 2 * dim * dim + 1; i++)
          tdyn_pd_parameters.push_back(parameters.at(i));
      }

      // ensure that the desired inertia matrix is positive definite (should also
      // check for symmetry actually)
      Eigen::LLT<Eigen::MatrixXd> lltOfB(
          B); // compute the Cholesky decomposition of B
      if (lltOfB.info() == Eigen::NumericalIssue) {
        printHiqpWarning("The given desired inertia matrix B is not positive "
                         "definite! Initialization of TDynImpedance object "
                         "failed!");

        return -1;
      }

      // Checking for symmetry
      if (B != B.transpose()){
        printHiqpWarning("The given desired inertia matrix B is not symmetric."
                          "Initialization of TDynImpedance object "
                          "failed.");
        return -1;
      }

      B_inv_ = pinv(B);


      double temp1 = std::stod(parameters.at(3));
      if ((k_d_ /temp1) < 2.0 * sqrt(k_p_ * temp1)){
        printHiqpWarning("The system is underdamped");
      }

      // // // initialize the controller through initialization of the base class
      // if (TDynPDController::init(tdyn_pd_parameters, robot_state, e_dot_initial,
      //                            e_dot_initial, e_final, e_dot_final) != 0){
      //                              return -1;
      //                            }


      // TODO change it to init through base class
      e_ddot_star_ = B_inv_ * (-k_p_ * e_initial - k_d_ * e_dot_initial);

      /* ============================ DESTROY
      */
      nh_ = ros::NodeHandle();

      return 0;
    }

// =================================================================
  void TDynImpedance::FTcallback(const geometry_msgs::WrenchStamped& msg){
      Eigen::Matrix<double, 6, 1> Wrench_fts_frame;

      Wrench_fts_frame << msg.wrench.force.x,  msg.wrench.force.y,
                          msg.wrench.force.z,  msg.wrench.torque.x,
                          msg.wrench.torque.y, msg.wrench.torque.z;
      // tf::wrenchMsgtoEigen(msg, Wrench_fts_frame)

      // TODO 1) get rotation matrix so F_tip = Rotation_matrix * F_measured
     //       2) get the normal plane, F_task = n.transpose * n * F_point
     //
  }


  int TDynImpedance::update(const RobotStatePtr robot_state,
                                  const std::shared_ptr<TaskDefinition> def) {
      // System equation
      // e_ddot_star_ =  B_inv_ * (-k_p_ * def->getTaskValue() - k_d_ * def->getTaskDerivative() +
      //                 def->getExogeneousTaskQuantities());
      e_ddot_star_ =  B_inv_ * (-k_p_ * def->getTaskValue() - k_d_ * def->getTaskDerivative() +
                      measured_F);
      return 0;
     }

  int TDynImpedance::monitor() { return 0; }
          } // ns tasks
} // ns hiqp
PLUGINLIB_EXPORT_CLASS(hiqp::tasks::TDynImpedance, hiqp::TaskDynamics)
