#include <gz/sim/EventManager.hh>  //Library for event manager (If put down of gz/system intellisense doesnt detect it)
#include <gz/sim/System.hh>        //Libraries for plugin (ISystem....)
#include <gz/plugin/Register.hh>   //To register the plugin
#include <gz/transport/Node.hh>    //For communications (Topics)
#include <gz/utils.hh>
#include <gz/sim/Model.hh>  //To control models (The robot)
#include <gz/math/Vector3.hh>
#include <gz/math/PID.hh>
#include <gz/msgs/float_v.pb.h>
#include <gz/sim/components.hh>  //To get the components
#include <gz/sim/Joint.hh>

// Inherit from System and 2 extra interfaces:
// ISystemConfigure and ISystemPostUpdate
class JointPlugin : public gz::sim::System, public gz::sim::ISystemConfigure, public gz::sim::ISystemUpdate
{
private:
  // Gz transport node
  gz::transport::Node node;
  // Model (robot)
  gz::sim::Model model;
  // Revolute joints
  std::vector<gz::sim::Joint> joints;
  // Angles -> Joints position (in radians)
  std::vector<float> targetAngles;
  // Pid controllers
  std::vector<gz::math::PID> pidControllers;
  // Velocity to calculate pid
  std::vector<double> previousVelocity;

  void OnRosMsg(const gz::msgs::Float_V& _msg)
  {
    // Store the received data (assuming it contains 12 angles for simplicity)
    if (_msg.data().size() == 12)
    {
      for (std::size_t i = 0; i < 12; ++i)
      {
        // Converting degrees to radians since gz works with rads
        // Note:: The real servos work with degrees
        //To do the conversion we also substract 90 degrees since the servo uses 180 degrees total but the simulation goes from -90 to 90
        this->targetAngles[i] = (_msg.data()[i] - 90)* (M_PI / 180);

      }
    }
  }

public:
  JointPlugin() = default;
  ~JointPlugin() override = default;

  // ISystemConfigure override
  virtual void Configure(const gz::sim::Entity& _entity, const std::shared_ptr<const sdf::Element>& _sdf,
                         gz::sim::EntityComponentManager& _ecm, gz::sim::EventManager& _eventMgr)
  {
    (void)_sdf;
    (void)_eventMgr;

    // Create model object to access convenient functions
    this->model = gz::sim::Model(_entity);

    if (!model.Valid(_ecm))
    {
      gzerr << "JointControllerPlugin should be attached to a valid model entity. "
            << "Failed to initialize." << std::endl;
      return;
    }

    // Retrieve joints
    auto allJoints = model.Joints(_ecm);

    // Filter for revolute joints and limit to the first 12
    int revoluteJointCount = 0;
    for (const auto& jointEntity : allJoints)
    {
      auto joint = gz::sim::Joint(jointEntity);
      if (joint.Type(_ecm) == sdf::JointType::REVOLUTE)
      {
        this->joints.push_back(joint);
        revoluteJointCount++;
        if (revoluteJointCount == 12)
        {
          break;
        }
      }
    }

    if (this->joints.size() != 12)
    {
      gzerr << "Expected 12 revolute joints, but found " << this->joints.size() << std::endl;
      return;
    }

    // Gz node
    this->node.Subscribe("/Joints_angle", &JointPlugin::OnRosMsg, this);

    // Iniatalizing velocity and angles
    this->previousVelocity.resize(12, 0.0);
    this->targetAngles.resize(12, 0.0);

    // Set PID gains
    double kp = 3;  // Proportional gain
    double ki = 0.1;
    double kd = 0.01;  // Derivative gain

    // Initialize the PID controllers for each joint
    for (int i = 0; i < 12; ++i)
    {
      this->pidControllers.push_back(gz::math::PID(kp, ki, kd));
    }
  }

  // ISystemUpdate
  virtual void Update(const gz::sim::UpdateInfo& _info, gz::sim::EntityComponentManager& _ecm)
  {
    // If simulation isn/t running
    if (_info.paused)
      return;
    // If simulation hasn't started
    if (_info.simTime <= std::chrono::_V2::steady_clock::duration{ 0 })
    {
      // Note that this basically makes sure that the values retrieved by position and velocity are well initialized
      return;  // Wait until the simulation has started
    }

    for (std::size_t i = 0; i < 12; ++i)
    {
      this->joints[i].EnablePositionCheck(_ecm);
      this->joints[i].EnableVelocityCheck(_ecm);
      auto currentAngleOpt = this->joints[i].Position(_ecm);
      auto currentVelocityOpt = this->joints[i].Velocity(_ecm);

      // Check if the values are valid
      if (!currentAngleOpt.has_value() || !currentVelocityOpt.has_value())
      {
        gzwarn << "Joint " << i << " has no valid position or velocity." << std::endl;
        continue;
      }

      // Check that the vectors have at least one element
      if (currentAngleOpt.value().empty() || currentVelocityOpt.value().empty())
      {
        gzwarn << "Joint " << i << " returned an empty position or velocity vector." << std::endl;
        continue;
      }

      if (this->pidControllers.size() <= i)
      {
        gzerr << "PID controller index " << i << " out of bounds." << std::endl;
        return;
      }

      double currentAngle = currentAngleOpt.value()[0];  // Assuming single DOF joints
      double currentVelocity = currentVelocityOpt.value()[0];

      // Apply a low-pass filter to the velocity
      double alpha = 0.8;
      double smoothedVelocity = alpha * currentVelocity + (1 - alpha) * this->previousVelocity[i];
      this->previousVelocity[i] = smoothedVelocity;

      // Calculating error
      double error = currentAngle - this->targetAngles[i];

      // Calculating force through PID
      double force = this->pidControllers[i].Update(error, smoothedVelocity, _info.dt);

      gzdbg << "Joint " << i << ": Error: " << error << ", Force: " << force << std::endl;

      // Max force equal to the max torque of the servo dm996
      double maxForce = 1.275;

      // Safety limits
      force = std::clamp(force, -maxForce, maxForce);

      this->joints[i].SetForce(_ecm, { force });
    }


    // std::vector<double> torque = {-0.2};
    // std::vector<double> torque_R = {0.2};
    // std::vector<double> torque_2 = {0.1};
    // std::vector<double> torque_low = {0.04};
  }
};

// Register plugin
GZ_ADD_PLUGIN(JointPlugin, gz::sim::System, JointPlugin::ISystemConfigure, JointPlugin::ISystemUpdate)

// Add plugin alias so that we can refer to the plugin without the version
GZ_ADD_PLUGIN_ALIAS(JointPlugin, "gz::sim::JointPlugin")
