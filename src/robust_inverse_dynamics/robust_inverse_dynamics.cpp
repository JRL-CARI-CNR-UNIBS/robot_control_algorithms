#include <robust_inverse_dynamics/robust_inverse_dynamics.h>

namespace robot_control
{

bool RobustInverseDynamics::init(ros::NodeHandle& controller_nh, const rosdyn::ChainPtr &chain)
{

  m_controller_nh=controller_nh;
  m_chain=chain;

  m_nax=m_chain->getJointsNumber();

  // set natural frequency and damping
  double wn, xi;
  if (!m_controller_nh.getParam("natural_frequency",wn))
  {
    ROS_ERROR("Position gain Kp is not set");
    return false;
  }
  if (!m_controller_nh.getParam("damping",xi))
  {
    ROS_ERROR("Velocity gain Kd is not set");
    return false;
  }
  m_Kp=wn*wn;
  m_Kd=2.0*xi*wn;

  double Ki;
  if (!m_controller_nh.getParam("integral_gain",Ki))
  {
    ROS_INFO("Integral gain is not set, use 0");
    Ki=0.0;
  }


  if (!m_controller_nh.getParam("robustness_gain",m_rho))
  {
    ROS_ERROR("robustness_gain is not set");
    return false;
  }

  m_acceleration.resize(m_nax);
  m_velocity.resize(m_nax);
  m_position.resize(m_nax);
  m_max_position.resize(m_nax);
  m_min_position.resize(m_nax);
  m_effort.resize(m_nax);
  m_max_effort.resize(m_nax);
  m_target_effort.resize(m_nax);
  m_xi.resize(m_nax);
  m_Ki.resize(m_nax);

  m_max_effort=m_chain->getTauMax();
  m_max_velocity=m_chain->getDQMax();
  m_max_position=m_chain->getQMax();
  m_min_position=m_chain->getQMin();
  m_acceleration.setZero();
  m_velocity.setZero();
  m_position.setZero();
  m_effort.setZero();
  m_target_effort.setZero();
  m_Ki=m_max_effort*Ki;

  ROS_INFO("Controller '%s' well initialized",m_controller_nh.getNamespace().c_str());
  ROS_DEBUG("Kp=%f, Kd=%f",m_Kp,m_Kd);


  return true;
}

void RobustInverseDynamics::starting(const Eigen::VectorXd &q, const Eigen::VectorXd &qd)
{
  m_configured = false;

  m_position=q;
  m_velocity=qd;
  m_effort.setZero();
  m_xi.setZero();
}

void RobustInverseDynamics::stopping()
{
  ROS_INFO("[ %s ] Stopping controller", m_controller_nh.getNamespace().c_str());
  m_configured = false;
}

void RobustInverseDynamics::update(const Eigen::VectorXd& q,
                                   const Eigen::VectorXd& qd,
                                   const Eigen::VectorXd &target_q,
                                   const Eigen::VectorXd &target_qd,
                                   const double &dt,
                                   Eigen::VectorXd& tau_command)
{
  m_position=q;
  m_velocity=qd;

  Eigen::VectorXd velocity_error=target_qd-m_velocity;
  Eigen::VectorXd positon_error=target_q-m_position;

  double velocity_norm=velocity_error.norm();
  if (velocity_norm<1e-4)
    m_robust_term=m_rho*1e4*velocity_error;
  else
    m_robust_term=m_rho*velocity_error/velocity_norm;

  m_acceleration=m_Kp*positon_error+m_Kd*velocity_error+m_robust_term;

  m_effort=m_chain->getJointTorque(m_position,m_velocity,m_acceleration)+m_xi;

  for (unsigned int iax=0;iax<m_nax;iax++)
  {
    double sat=std::max(-m_max_effort(iax),std::min(m_effort(iax),m_max_effort(iax)));
    if (sat==m_effort(iax))
      m_xi+=positon_error.cwiseProduct(m_Ki)*dt;
    m_effort(iax)=sat;
  }
  tau_command=m_effort;
}



}  //  namespace robot_control
