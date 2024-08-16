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

#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include <custom_action_interfaces/action/servo_legs.hpp>
#include <std_msgs/msg/multi_array_dimension.hpp>
#include <std_msgs/msg/multi_array_layout.hpp>
#include <iostream>

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
  // ROS 2 Node
  rclcpp::Node::SharedPtr rosNode;

   // Action server definition
  using JointAction = custom_action_interfaces::action::ServoLegs;
  rclcpp_action::Server<JointAction>::SharedPtr actionServer;

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

     // Initialize ROS 2 node
    if (!rclcpp::ok())
    {
      rclcpp::init(0, nullptr);
    }
    this->rosNode = std::make_shared<rclcpp::Node>("gazebo_joint_plugin");

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
    //Rearranging the vector order
    //This way it goes from lower to upper leg from left forward to right backward
    std::swap(this->joints[0],this->joints[2]);
    std::swap(this->joints[3],this->joints[5]);
    std::swap(this->joints[6],this->joints[8]);
    std::swap(this->joints[9],this->joints[11]);

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
    double kp = 10;  // Proportional gain
    double ki = 3;
    double kd = 0.01;  // Derivative gain

    // Initialize the PID controllers for each joint
    for (int i = 0; i < 12; ++i)
    {
      this->pidControllers.push_back(gz::math::PID(kp, ki, kd));
    }

    // Initialize ROS 2 action server
    this->actionServer = rclcpp_action::create_server<JointAction>(
        this->rosNode,
        "ServoLegs",
        std::bind(&JointPlugin::handle_goal, this, std::placeholders::_1, std::placeholders::_2),
        std::bind(&JointPlugin::handle_cancel, this, std::placeholders::_1),
        std::bind(&JointPlugin::handle_accepted, this, std::placeholders::_1));

    // Start ROS 2 node in a separate thread
    std::thread([this]() { rclcpp::spin(this->rosNode); }).detach();
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

      //Miuzei9g havee less torque
      double maxForce = 0.215;

      if(i == 2 || i == 5 || i == 8 || i == 11){
        // Max force equal to the max torque of the servo dm996
        maxForce = 1.275;
      }
      // Safety limits
      force = std::clamp(force, -maxForce, maxForce);

      this->joints[i].SetForce(_ecm, { force });
    }

  }

  // Handle goal
  rclcpp_action::GoalResponse handle_goal(const rclcpp_action::GoalUUID &uuid, std::shared_ptr<const JointAction::Goal> goal)
  {
    RCLCPP_INFO(this->rosNode->get_logger(), "Received goal request");
    (void)uuid;

    if (goal->angles.size() % 12 != 0)
    {
      RCLCPP_WARN(this->rosNode->get_logger(), "Goal has incorrect number of angles");
      return rclcpp_action::GoalResponse::REJECT;
    }

    return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
  }

  // Handle cancel
  rclcpp_action::CancelResponse handle_cancel(const std::shared_ptr<rclcpp_action::ServerGoalHandle<JointAction>> goal_handle)
  {
    RCLCPP_INFO(this->rosNode->get_logger(), "Received request to cancel goal");
    return rclcpp_action::CancelResponse::ACCEPT;
  }

  // Handle accepted
  void handle_accepted(const std::shared_ptr<rclcpp_action::ServerGoalHandle<JointAction>> goal_handle)
  {
    std::thread([this, goal_handle]()
    {
      const auto goal = goal_handle->get_goal();
      auto result = std::make_shared<JointAction::Result>();

      //Retrieving layout and data
      const auto &layout = goal->layout;
      const auto &angles = goal->angles;

      // Extract the dimensions from the layout
      size_t rows = layout.dim[0].size;
      size_t cols = layout.dim[1].size;

      // Reconstruct the 2D array
      std::vector<std::vector<uint8_t>> array_2d(rows, std::vector<uint8_t>(cols));

      for (size_t i = 0; i < rows; i++) {
        for (size_t j = 0; j < cols; j++) {
              array_2d[i][j] = angles[(i * cols) + j];
          }
      }

      for (std::size_t i = 0; i < rows; i++)
      {
        for(std::size_t j = 0; j < cols; j++){
          if (goal_handle->is_canceling())
          {
            result->success = false;
            goal_handle->canceled(result);
            RCLCPP_INFO(this->rosNode->get_logger(), "Goal canceled");
            return;
          }

          this->targetAngles[j] = (array_2d[i][j] - 90) * (M_PI / 180);
          std::this_thread::sleep_for(std::chrono::milliseconds(500));  // Simulate processing delay
        }
      }

      result->success = true;
      goal_handle->succeed(result);
      RCLCPP_INFO(this->rosNode->get_logger(), "Goal succeeded");

    }).detach();
  }
};

// Register plugin
GZ_ADD_PLUGIN(JointPlugin, gz::sim::System, JointPlugin::ISystemConfigure, JointPlugin::ISystemUpdate)

// Add plugin alias so that we can refer to the plugin without the version
GZ_ADD_PLUGIN_ALIAS(JointPlugin, "gz::sim::JointPlugin")
