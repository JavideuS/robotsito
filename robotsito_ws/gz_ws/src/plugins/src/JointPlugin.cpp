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
//Ros libraries
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include <custom_action_interfaces/action/servo_legs.hpp>
#include <std_msgs/msg/multi_array_dimension.hpp>
#include <std_msgs/msg/multi_array_layout.hpp>
#include <iostream>
//Multithreading libraries
#include <thread>
#include <mutex>
#include <condition_variable>
#include <ThreadPool.h>

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

  // Mutex for synchronizing access to shared data
  std::mutex mutex_;

  // Condition variable to signal when all threads are done
  std::condition_variable cv_;
  int threadsRemaining_;
  //Threadpool
  ThreadPool pool{6};  // Example: Construct with a number of threads


   // Action server definition
  using JointAction = custom_action_interfaces::action::ServoLegs;
  rclcpp_action::Server<JointAction>::SharedPtr actionServer;

  void OnRosMsg(const gz::msgs::Float_V& _msg)
  {
    // Store the received data (assuming it contains 18 angles for simplicity)
    if (_msg.data().size() == 18)
    {
      for (std::size_t i = 0; i < 18; ++i)
      {
        // Converting degrees to radians since gz works with rads
        // Note:: The real servos work with degrees
        //To do the conversion we also substract 90 degrees since the servo uses 180 degrees total but the simulation goes from -90 to 90
        this->targetAngles[i] = (_msg.data()[i] - 90)* (M_PI / 180);

      }
    }
  }

  // Method to handle moving a single leg
 void MoveLeg(std::size_t leg_index, const std::vector<std::vector<double>>& leg_angles, size_t num_iterations, std::shared_ptr<rclcpp_action::ServerGoalHandle<JointAction>> goal_handle)
{
    for (size_t iteration = 0; iteration < num_iterations; ++iteration)
    {
        for (const auto& joint_angles : leg_angles)
        {
            if (goal_handle->is_canceling())
            {
                return;  // Exit if the goal is canceled
            }

            {
                // Lock the mutex while accessing shared data
                std::lock_guard<std::mutex> lock(mutex_);
                for (std::size_t joint = 0; joint < joint_angles.size(); ++joint)
                {
                    // Update the target angle for the joint in this leg
                    this->targetAngles[leg_index * 3 + joint] = (joint_angles[joint] - 90) * (M_PI / 180);
                }
            }

            std::this_thread::sleep_for(std::chrono::milliseconds(500));  // Simulate processing delay
        }
    }

    {
        // Signal that this task is done
        std::lock_guard<std::mutex> lock(mutex_);
        if (--threadsRemaining_ == 0)
        {
            cv_.notify_one();  // Notify the main thread that all leg threads are done
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

    // Filter for revolute joints and limit to the first 18
    int revoluteJointCount = 0;
    for (const auto& jointEntity : allJoints)
    {
      auto joint = gz::sim::Joint(jointEntity);
      if (joint.Type(_ecm) == sdf::JointType::REVOLUTE)
      {
        this->joints.push_back(joint);
        revoluteJointCount++;
        if (revoluteJointCount == 18)
        {
          break;
        }
      }
    }

    if (this->joints.size() != 18)
    {
      gzerr << "Expected 18 revolute joints, but found " << this->joints.size() << std::endl;
      return;
    }

    // Gz node
    this->node.Subscribe("/Joints_angle", &JointPlugin::OnRosMsg, this);

    // Iniatalizing velocity and angles
    this->previousVelocity.resize(18, 0.0);
    this->targetAngles.resize(18, 0.0);

    // Set PID gains
    double kp = 3;  // Proportional gain
    double ki = 1;
    double kd = 0.01;  // Derivative gain

    // Initialize the PID controllers for each joint
    for (int i = 0; i < 18; ++i)
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

    for (std::size_t i = 0; i < 18; ++i)
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

      // Max force equal to the max torque of the miuzei9g 
      double maxForce = 0.216;

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

    if (goal->angles.size() % 18 != 0)
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

        // Retrieve layout, angles data, and number of iterations
        const auto &layout = goal->layout;
        const auto &angles = goal->angles;
        size_t num_iterations = goal->iterations;  // Ensure that your action definition includes this field

        // Extract the dimensions from the layout
        size_t num_legs = layout.dim[0].size;
        size_t num_time_steps = layout.dim[1].size;
        size_t num_joints_per_leg = 3;

        // Reconstruct the 3D array (leg, time_step, joint_angle)
        std::vector<std::vector<std::vector<double>>> angles_3d(num_legs, std::vector<std::vector<double>>(num_time_steps, std::vector<double>(num_joints_per_leg)));
        for (size_t i = 0; i < num_legs; ++i)
        {
            for (size_t j = 0; j < num_time_steps; ++j)
            {
                for (size_t k = 0; k < num_joints_per_leg; ++k)
                {
                    angles_3d[i][j][k] = angles[i * num_time_steps * num_joints_per_leg + j * num_joints_per_leg + k];
                }
            }
        }

        // Set the number of threads remaining
        {
            std::lock_guard<std::mutex> lock(mutex_);
            threadsRemaining_ = num_legs;
        }

        // Launch tasks for each leg
        for (std::size_t i = 0; i < num_legs; ++i)
        {
            pool.enqueue([this, i, angles_3d = angles_3d[i], num_iterations, goal_handle]()
            {
                this->MoveLeg(i, angles_3d, num_iterations, goal_handle);
            });
        }

        // Wait for all tasks to complete
        {
            std::unique_lock<std::mutex> lock(mutex_);
            cv_.wait(lock, [this]() { return threadsRemaining_ == 0; });
        }

        // Check if goal was canceled
        if (goal_handle->is_canceling())
        {
            result->success = false;
            goal_handle->canceled(result);
            RCLCPP_INFO(this->rosNode->get_logger(), "Goal canceled");
            return;
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
