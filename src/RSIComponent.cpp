#include <rtt/RTT.hpp>
#include <rtt/TaskContext.hpp>
#include <rtt/Component.hpp>
#include <rtt/Logger.hpp>

#include <rtt_rsi/rsi_command.h>
#include <rtt_rsi/rsi_state.h>
#include <rtt_rsi/udp_server.h>

namespace rtt_rsi
{
static const double RAD2DEG = 57.295779513082323;
static const double DEG2RAD = 0.017453292519943295;

class RSIComponent : public RTT::TaskContext
{
public:
  RSIComponent(const std::string& name)
    : RTT::TaskContext(name)
    , local_host_("127.0.0.1")
    , local_port_(49152)
    , joint_position_(6, 0.0)
    , joint_velocity_(6, 0.0)
    , joint_effort_(6, 0.0)
    , joint_position_command_(6, 0.0)
    , joint_velocity_command_(6, 0.0)
    , joint_effort_command_(6, 0.0)
    , joint_names_(6)
    , rsi_initial_joint_positions_(6, 0.0)
    , rsi_joint_position_corrections_(6, 0.0)
    , ipoc_(0)
    , n_dof_(6)
  {
    RTT::log(RTT::Info) << "Constructing ! " << RTT::endlog();

    this->ports()->addPort("JointPositionCommand", port_joint_position_command_).doc("");
    this->ports()->addPort("JointPosition", port_joint_position_).doc("");

    in_buffer_.resize(1024);
    out_buffer_.resize(1024);
    remote_host_.resize(1024);
    remote_port_.resize(1024);
  }

  bool configureHook()
  {
    RTT::log(RTT::Info) << "Configuring  ! " << RTT::endlog();

    port_joint_position_.setDataSample(joint_position_);

    RTT::log(RTT::Info) << "Setting up RSI server on: (" << local_host_ << ", " << local_port_ << ")" << RTT::endlog();
    server_.reset(new UDPServer(local_host_, local_port_));
    return true;
  }

  bool startHook()
  {
    RTT::log(RTT::Info) << "Starting  ! " << RTT::endlog();
    RTT::log(RTT::Info) << "Waiting for robot!" << RTT::endlog();

    int bytes = server_->recv(in_buffer_);

    // Drop empty <rob> frame with RSI <= 2.3
    if (bytes < 100)
    {
      bytes = server_->recv(in_buffer_);
    }

    rsi_state_ = RSIState(in_buffer_);
    for (std::size_t i = 0; i < n_dof_; ++i)
    {
      joint_position_[i] = DEG2RAD * rsi_state_.positions[i];
      joint_position_command_[i] = joint_position_[i];
      rsi_initial_joint_positions_[i] = rsi_state_.initial_positions[i];
    }
    ipoc_ = rsi_state_.ipoc;
    out_buffer_ = RSICommand(rsi_joint_position_corrections_, ipoc_).xml_doc;
    server_->send(out_buffer_);

    // Set receive timeout to 1 second
    server_->set_timeout(1000);
    RTT::log(RTT::Info) << "Got connection from robot!" << RTT::endlog();

    return true;
  }

  void updateHook()
  {
    // RTT::log(RTT::Info) << "Updating ! " << RTT::endlog();

    if (this->isRunning())
    {
      // Read
      in_buffer_.resize(1024);

      if (server_->recv(in_buffer_) == 0)
      {
        this->error();
      }

      rsi_state_ = RSIState(in_buffer_);
      for (std::size_t i = 0; i < n_dof_; ++i)
      {
        joint_position_[i] = DEG2RAD * rsi_state_.positions[i];
      }
      ipoc_ = rsi_state_.ipoc;

      port_joint_position_command_.read(joint_position_command_);

      RTT::log(RTT::Info) << joint_position_command_[1] << RTT::endlog();

      // Write
      out_buffer_.resize(1024);

      for (std::size_t i = 0; i < n_dof_; ++i)
      {
        rsi_joint_position_corrections_[i] = (RAD2DEG * joint_position_command_[i]) - rsi_initial_joint_positions_[i];
      }

      out_buffer_ = RSICommand(rsi_joint_position_corrections_, ipoc_).xml_doc;
      server_->send(out_buffer_);
    }
    this->trigger();
  }

private:
  RTT::InputPort<std::vector<double>> port_joint_position_command_;
  RTT::OutputPort<std::vector<double>> port_joint_position_;

  unsigned int n_dof_;

  std::vector<std::string> joint_names_;

  std::vector<double> joint_position_;
  std::vector<double> joint_velocity_;
  std::vector<double> joint_effort_;
  std::vector<double> joint_position_command_;
  std::vector<double> joint_velocity_command_;
  std::vector<double> joint_effort_command_;

  // RSI
  RSIState rsi_state_;
  RSICommand rsi_command_;
  std::vector<double> rsi_initial_joint_positions_;
  std::vector<double> rsi_joint_position_corrections_;
  unsigned long long ipoc_;

  std::unique_ptr<UDPServer> server_;
  std::string local_host_;
  int local_port_;
  std::string remote_host_;
  std::string remote_port_;
  std::string in_buffer_;
  std::string out_buffer_;
};

}  // namespace rtt_rsi

ORO_CREATE_COMPONENT(rtt_rsi::RSIComponent)