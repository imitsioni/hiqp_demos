#include <hiqp/tasks/tdef_ft.h>
#include <iterator>
#include <sstream>
#include <pluginlib/class_list_macros.h>

namespace hiqp {
namespace tasks {
//==================================================================================
TDefFT::TDefFT(std::shared_ptr<GeometricPrimitiveMap> geom_prim_map,
                     std::shared_ptr<Visualizer> visualizer)
  : TaskDefinition(geom_prim_map, visualizer) {}

TDefFT::~TDefFT() noexcept {}


int TDefFT::init(const std::vector<std::string> &parameters,
                    RobotStatePtr robot_state) {
  //make sure dimensions are correct (the first parameter is the task definition name)
  int size = parameters.size();

  // Using 5 params so the sensor frame is explicitly imported
  if (size != 5) {
    printHiqpWarning("TDefFT requires 5 parameters, got " +
                     std::to_string(size) + "! Initialization failed!");
    return -1;
  }

  // Initialization of the base class (geometric projection) ------

  std::stringstream ss(parameters.at(4));
  std::vector<std::string> args(std::istream_iterator<std::string>{ss},
  std::istream_iterator<std::string>{});
  sensor_frame_ = parameters.at(1); //read the sensor frame

  // Subscribe to the wrench topic
  nh_ = ros::NodeHandle();
  wrench_sub = nh_.subscribe("wrench", 10, &TDefFT::FTcallback, this);

  task_signs_.insert(task_signs_.begin(), 1, 0); //equality task (0)
  performance_measures_.resize(0);  //no custom performance measure needed

  unsigned int n_joints = robot_state->getNumJoints();
  //Our operational space is the scalar along the x-axis, therefore task error/error derivative are scalar and the Jacobian/Jacobian derivative are in \mathbb{R}^{1 \times n}
  e_ = Eigen::VectorXd::Zero(1);
  e_dot_ = Eigen::VectorXd::Zero(1);
  J_ = Eigen::MatrixXd::Zero(1,n_joints);
  J_dot_ = Eigen::MatrixXd::Zero(1,n_joints);
  f_ = Eigen::VectorXd::Zero(1); //f is a possible exogeneous task quantity, e.g., for external force

  //set up forward kinematics and Jacobian solvers using the kinematic tree read from the urdf file
  fk_solver_pos_ =
      std::make_shared<KDL::TreeFkSolverPos_recursive>(robot_state->kdl_tree_);
  fk_solver_jac_ =
      std::make_shared<KDL::TreeJntToJacSolver>(robot_state->kdl_tree_);

  std::shared_ptr<GeometricPrimitiveMap> gpm = this->getGeometricPrimitiveMap();

  primitive_a_ = gpm->getGeometricPrimitive<GeometricPoint>(args.at(0));
  if (primitive_a_ == nullptr) {
      printHiqpWarning(
  	 "In TDefGeometricProjection::init(), couldn't find primitive with name "
  	 "'" +
  	 args.at(0) + "'. Unable to create task!");
     return -3;
      }

  primitive_b_ = gpm->getGeometricPrimitive<GeometricPlane>(args.at(2));
  if (primitive_b_ == nullptr) {
      printHiqpWarning(
  	 "In TDefGeometricProjection::init(), couldn't find primitive with name "
  	 "'" +
  	 args.at(2) + "'. Unable to create task!");
     return -3;
      }

  gpm->addDependencyToPrimitive(args.at(0), this->getTaskName());
  gpm->addDependencyToPrimitive(args.at(2), this->getTaskName());

  return 0; //returning 0 indicates success
}


//========================================================================
int TDefFT::update(RobotStatePtr robot_state) {
 // Updating the base class (geometric projection)
 int retval = 0;

  retval = fk_solver_pos_->JntToCart(robot_state->kdl_jnt_array_vel_.q, pose_a_,
       primitive_a_->getFrameId());


  if (retval != 0) {
  std::cerr << "In TDefGeometricProjection::update : Can't solve position "
            << "of link '" << primitive_a_->getFrameId() << "'"
            << " in the "
            << "KDL tree! KDL::TreeFkSolverPos_recursive::JntToCart return "
            << "error code '" << retval << "'\n";
          return -1;
    }

  retval = fk_solver_pos_->JntToCart(robot_state->kdl_jnt_array_vel_.q, pose_b_,
       primitive_b_->getFrameId());
  if (retval != 0) {
  std::cerr << "In TDefGeometricProjection::update : Can't solve position "
            << "of link '" << primitive_b_->getFrameId() << "'"
            << " in the "
            << "KDL tree! KDL::TreeFkSolverPos_recursive::JntToCart return "
            << "error code '" << retval << "'\n";
          return -2;
    }

    jacobian_a_.resize(robot_state->kdl_jnt_array_vel_.q.rows());
    retval = fk_solver_jac_->JntToJac(robot_state->kdl_jnt_array_vel_.q,
        jacobian_a_, primitive_a_->getFrameId());
    if (retval != 0) {
  std::cerr << "In TDefGeometricProjection::update : Can't solve jacobian "
    << "of link '" << primitive_a_->getFrameId() << "'"
    << " in the "
    << "KDL tree! KDL::TreeJntToJacSolver return error code "
    << "'" << retval << "'\n";
  return -3;
    }

    jacobian_b_.resize(robot_state->kdl_jnt_array_vel_.q.rows());
    retval = fk_solver_jac_->JntToJac(robot_state->kdl_jnt_array_vel_.q,
        jacobian_b_, primitive_b_->getFrameId());
    if (retval != 0) {
  std::cerr << "In TDefGeometricProjection::update : Can't solve jacobian "
    << "of link '" << primitive_b_->getFrameId() << "'"
    << " in the "
    << "KDL tree! KDL::TreeJntToJacSolver return error code "
    << "'" << retval << "'\n";
  return -4;
    }


    jacobian_dot_a_.resize(robot_state->kdl_jnt_array_vel_.q.rows());
    retval = treeJntToJacDot(robot_state->kdl_tree_, jacobian_a_, robot_state->kdl_jnt_array_vel_,
           jacobian_dot_a_, primitive_a_->getFrameId());
    if (retval != 0) {
  std::cerr << "In TDefGeometricProjection::update : Can't solve jacobian derivative "
    << "of link '" << primitive_a_->getFrameId() << "'"
    << " in the "
    << "KDL tree! treeJntToJacDot return error code "
    << "'" << retval << "'\n";
  return -5;
    }

    jacobian_dot_b_.resize(robot_state->kdl_jnt_array_vel_.q.rows());
    retval = treeJntToJacDot(robot_state->kdl_tree_, jacobian_b_, robot_state->kdl_jnt_array_vel_,
           jacobian_dot_b_, primitive_b_->getFrameId());
    if (retval != 0) {
  std::cerr << "In TDefGeometricProjection::update : Can't solve jacobian derivative "
    << "of link '" << primitive_b_->getFrameId() << "'"
    << " in the "
    << "KDL tree! treeJntToJacDot return error code "
    << "'" << retval << "'\n";
  return -6;
 }
 // ======================================================================
 //               Projection of the base class
 // ======================================================================

   KDL::Vector p1__ = pose_a_.M * primitive_a_->getPointKDL(); //point 1 from link origin to ee expressed in the world frame
   KDL::Vector p1 = pose_a_.p + p1__; //absolute ee point 1 expressed in the world frame

   KDL::Vector n = pose_b_.M * primitive_b_->getNormalKDL();  //plane normal expressed in the world frame
   KDL::Vector k__ = primitive_b_->getOffset() * n; //point on the plane expressed in the world frame
   KDL::Vector v__ = k__ + n; //normal tip expressed in the world frame
   KDL::Vector k = k__ + pose_b_.p; // absolute point on the plane expressed in the world frame
   double b = dot(n,k); //plane offset in the world frame

   double q_nr=jacobian_a_.columns();
   KDL::Jacobian J_vk, J_p1k, J_dot_vk, J_dot_p1k, J_k, J_dot_k;
   J_k.resize(q_nr);
   J_dot_k.resize(q_nr);
   J_vk.resize(q_nr);
   J_p1k.resize(q_nr);
   J_dot_vk.resize(q_nr);
   J_dot_p1k.resize(q_nr);

   changeJacRefPoint(jacobian_b_, v__, J_vk);
   changeJacRefPoint(jacobian_b_, k__, J_k);
   J_vk.data=J_vk.data - J_k.data;

   changeJacRefPoint(jacobian_a_, p1__, J_p1k);
   J_p1k.data=J_p1k.data - J_k.data;

   changeJacDotRefPoint(jacobian_b_, jacobian_dot_b_, robot_state->kdl_jnt_array_vel_, v__, J_dot_vk);
   changeJacDotRefPoint(jacobian_b_, jacobian_dot_b_, robot_state->kdl_jnt_array_vel_, k__, J_dot_k);
   J_dot_vk.data = J_dot_vk.data - J_dot_k.data;

   changeJacDotRefPoint(jacobian_a_, jacobian_dot_a_, robot_state->kdl_jnt_array_vel_, p1__, J_dot_p1k);
   J_dot_p1k.data = J_dot_p1k.data - J_dot_k.data;

   e_(0)=dot(n,p1)-b;
   J_=Eigen::Map<Eigen::Matrix<double,1,3> >((p1-k).data)*J_vk.data.topRows<3>()+Eigen::Map<Eigen::Matrix<double,1,3> >(n.data)*J_p1k.data.topRows<3>();
   Eigen::VectorXd qdot=robot_state->kdl_jnt_array_vel_.qdot.data;
   e_dot_=J_*qdot;
   J_dot_=Eigen::Map<Eigen::Matrix<double,1,3> >((p1-k).data)*J_dot_vk.data.topRows<3>()+Eigen::Map<Eigen::Matrix<double,1,3> >(n.data)*J_dot_p1k.data.topRows<3>()+(J_p1k.data.topRows<3>()*qdot).transpose()*J_vk.data.topRows<3>()+(J_vk.data.topRows<3>()*qdot).transpose()*J_p1k.data.topRows<3>();
 // ======================================================================

  projectForces(this->primitive_a_, this-> primitive_b_, robot_state);


  //awkward fix to not let the contribution of J_dot get out of hand due to numerical issues with large joint velocities induced by singularities
    double tol=1e5;
    if(fabs((J_dot_*robot_state->kdl_jnt_array_vel_.qdot.data)(0)) > tol){
              J_dot_.setZero();
              e_dot_.setZero();
    }

  return 0;
 }

int TDefFT::projectForces(std::shared_ptr<GeometricPoint> point,
                    std::shared_ptr<GeometricPlane> plane,
                    const RobotStatePtr robot_state){

      // Eigen::Matrix<double,3,1> temp_force;
    Eigen::Vector3d temp_force, temp_fforce;
    // temp_force << Wrench_fts_frame(0), Wrench_fts_frame(1), Wrench_fts_frame(2);
    Eigen::Vector3d fforce = Eigen::Map<Eigen::Vector3d>((pose_a_.M * KDL::Vector(Wrench_fts_frame[0],Wrench_fts_frame[1], Wrench_fts_frame[2])).data); // measured force expressed in the world frame
    Eigen::Vector3d n = Eigen::Map<Eigen::Vector3d>((pose_b_.M * plane->getNormalKDL()).data); // plane normal expressed in the world frame

    Eigen::Matrix<double, 3,3> plane_projection;
    plane_projection = n * n.transpose(); // 3x3
    // // DEBUG
    std::cout << "------------------------------------------------------" <<'\n';
    std::cout<< "Plane normal " << n << '\n';
    std::cout<< "Plane projection matrix " << '\n';
    std::cout << plane_projection << '\n';
    std::cout << "Force (actual) I'm reading is " <<'\n';
    std::cout << Wrench_fts_frame.topRows(3) << '\n';
    temp_force = plane_projection * Wrench_fts_frame.topRows(3);

    temp_fforce = plane_projection * fforce;

    std::cout << "Force (projected) I'm reading is " <<'\n';
    std::cout << temp_force << '\n';
    std::cout << "Force (rotated, i think) I'm reading is " <<'\n';
    std::cout << fforce << '\n';
    // f_ = Eigen::Vector3d(temp_force);
    f_ << temp_fforce[2];
    std::cout << "Force I'm sending is " <<'\n';
    // std::cout << f_ << ',' << f_.size() << '\n';
    std::cout << f_ << "\n VS " << '\n';
    std::cout << temp_fforce << '\n';
    std::cout << "------------------------------------------------------" <<'\n';

    return 0;


}

// =================================================================
  void TDefFT::FTcallback(const geometry_msgs::WrenchStamped& msg){

      Wrench_fts_frame << msg.wrench.force.x,  msg.wrench.force.y,
                          msg.wrench.force.z,  msg.wrench.torque.x,
                          msg.wrench.torque.y, msg.wrench.torque.z;
      // tf::wrenchMsgtoEigen(msg, Wrench_fts_frame)

  }


//==================================================================================
  int TDefFT::monitor() { return 0; }
//==================================================================================




} // namespace tasks

} // namespace hiqp

PLUGINLIB_EXPORT_CLASS(hiqp::tasks::TDefFT, hiqp::TaskDefinition) //export the new task definition
