#pragma once

# include <eigen_state_space_systems/eigen_controllers.h>
# include <rosdyn_core/primitives.h>
# include <ros/ros.h>

namespace robot_control
{

class RobustInverseDynamics
{
public:
  bool init(ros::NodeHandle& controller_nh, const rosdyn::ChainPtr& chain);
  void update(const Eigen::VectorXd& q,
              const Eigen::VectorXd& qd,
              const Eigen::VectorXd& target_q,
              const Eigen::VectorXd& target_qd,
              Eigen::VectorXd& tau_command);
  void starting(const Eigen::VectorXd& q,
                const Eigen::VectorXd& qd);
  void stopping();

protected:

  rosdyn::ChainPtr m_chain;

  Eigen::VectorXd m_position;
  Eigen::VectorXd m_velocity;
  Eigen::VectorXd m_acceleration;
  Eigen::VectorXd m_effort;

  Eigen::VectorXd m_target_effort;
  Eigen::VectorXd m_robust_term;
  Eigen::VectorXd m_max_effort;
  Eigen::VectorXd m_max_velocity;
  Eigen::VectorXd m_max_position;
  Eigen::VectorXd m_min_position;


  double m_Kp;
  double m_Kd;
  double m_rho; //  robustness gain


  ros::NodeHandle m_controller_nh;
  std::size_t m_nax;
  bool m_configured;
};
typedef std::shared_ptr<RobustInverseDynamics> RobustInverseDynamicsPtr;

}
