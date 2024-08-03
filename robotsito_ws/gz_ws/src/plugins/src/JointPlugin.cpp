#include <gz/sim/EventManager.hh> //Library for event manager (If put down of gz/system intellisense doesnt detect it)
#include <gz/sim/System.hh> //Libraries for plugin (ISystem....)
#include <gz/plugin/Register.hh> //To register the plugin
#include <gz/transport/Node.hh> //For communications (Topics)
#include <gz/common.hh>
#include <gz/sim/Model.hh> //To control models (The robot)
#include <gz/math/Vector3.hh>
#include <vector>
#include <gz/msgs/float_v.pb.h>
#include <gz/sim/components.hh>  //To get the components
#include <gz/sim/Joint.hh>
#include <iostream>


// Inherit from System and 2 extra interfaces:
// ISystemConfigure and ISystemPostUpdate
class JointPlugin : public gz::sim::System,
                    public gz::sim::ISystemConfigure,
                    public gz::sim::ISystemUpdate
{
  private:
    // Gz transport node
    gz::transport::Node node;
    //Model (robot)
    gz::sim::Model model;
    std::vector<gz::sim::Joint> joints;
    std::vector<float> currentAngles;
    std::vector<float> targetAngles;
    bool new_msg;

  void OnRosMsg(const gz::msgs::Float_V &_msg)
    {
    // Store the received data (assuming it contains 12 angles for simplicity)
      if (_msg.data().size() == 12) {
        for (std::size_t i = 0; i < this->targetAngles.size(); ++i)
        {
          //Converting degrees to radians since gz works with rads
          //Note:: The real servos work with degrees
          this->targetAngles[i] = _msg.data()[i] * (M_PI / 180) ;

          //Left side or moving the servo backward will represent negatives angles
          //So the simulator will work with 180 degrees freedom per joint
          //Which range from -1.5708 rads to 1.5708 rads
          // if (_msg.data()[i] < 90){
          //   this->targetAngles[i] *= -1;
          // }
        }
      }
      this->new_msg = true;
    }

  public:
    JointPlugin() = default;
    ~JointPlugin() override = default;

    //ISystemConfigure override
    virtual void 	Configure(const gz::sim::Entity &_entity, 
                            const std::shared_ptr< const sdf::Element > &_sdf, 
                            gz::sim::EntityComponentManager &_ecm, 
                            gz::sim::EventManager &_eventMgr){
    
    
    (void)_sdf;
    (void)_eventMgr;

    //Initializing new_msg to false
    this->new_msg=false;
    //Initializing currentangles position to 0
    this->currentAngles = {0,0,0,0,0,0,0,0,0,0,0,0};


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
    for (const auto &jointEntity : allJoints)
    { 
      auto joint = gz::sim::Joint(jointEntity);
      if (joint.Type(_ecm) == sdf::JointType::REVOLUTE)
      {
        this->joints.push_back(joint);
        revoluteJointCount++;
        if (revoluteJointCount == 12) {
          break;
        }
      }
    }

    if (this->joints.size() != 12)
    {
      gzerr << "Expected 12 revolute joints, but found " << this->joints.size() << std::endl;
      return;
    }
    this->targetAngles.resize(this->joints.size(), 0.0f);
    // Gz node
    this->node.Subscribe("/Joints_angle", &JointPlugin::OnRosMsg, this);
    }

    //ISystemUpdate
    virtual void Update 	( 	const gz::sim::UpdateInfo &    _info,
                          gz::sim::EntityComponentManager  & _ecm ) {
      
      if(this->new_msg){
        gzdbg << "Another msg" << std::endl;
        // std::vector<double> velocities = {5.0};
        // this->joints[0].ResetVelocity(_ecm,velocities);
        // this->joints[2].ResetVelocity(_ecm,velocities);
        // this->joints[3].ResetVelocity(_ecm,velocities);
        // this->joints[1].SetVelocity(_ecm,velocities);
        // velocities = {-1.0};
        // this->joints[4].SetVelocity(_ecm,velocities);
        // this->joints[5].SetVelocity(_ecm,velocities);
        // this->joints[6].SetVelocity(_ecm,velocities);
        // this->joints[7].SetVelocity(_ecm,velocities);

        std::vector<double> torque = {100.275};

        this->joints[0].SetForce(_ecm,torque);
        this->joints[1].SetForce(_ecm,torque);
        this->joints[2].SetForce(_ecm,torque);

        std::cout << "Message jaja\n";
       
        gzdbg << "Joint axis: " << this->joints[0].Axis(_ecm).value().data();
      //   for(std::size_t i = 0; i < joints.size(); i++){
      //     //) by default so if the targetAngle equals the currentAngle, it doesn't change position
      //     std::vector<double> velocities = {0.0};
      //     if(this->currentAngles[i] > this->targetAngles[i]){
      //      velocities = {5.0};
      //     }
      //     else if(this->currentAngles[i] < this->targetAngles[i]){
      //       velocities = {-5.0};
      //     }


      //     this->joints[i].SetVelocity(_ecm, velocities);
      //     auto velocity = this->joints[i].Velocity(_ecm);
          
      //    //Target angles becames current angles
      //    this->currentAngles[i] = this->targetAngles[i];

      //     //Debugging messages
      //     // Print joint name
      //     auto joint_name = this->joints[i].Name(_ecm);
      //     if (joint_name.has_value())
      //     {
      //       gzdbg << "Joint name: " << joint_name.value() << std::endl;
      //     }

      //     // Print current velocity
      //     auto joint_velocity = this->joints[i].Velocity(_ecm);
      //     this->joints[i].EnableVelocityCheck(_ecm);
      //     if (joint_velocity.has_value())
      //     {
      //       gzdbg << "Current velocity: " << joint_velocity.value().data() << std::endl;
      //     }

      //   }
         //Resetting the msg_sign
         this->new_msg=false; 
      } 
    }
};

// Register plugin
GZ_ADD_PLUGIN(JointPlugin,
              gz::sim::System,
              JointPlugin::ISystemConfigure,
              JointPlugin::ISystemUpdate)
 
// Add plugin alias so that we can refer to the plugin without the version
// namespace
GZ_ADD_PLUGIN_ALIAS(JointPlugin, "gz::sim::JointPlugin")


