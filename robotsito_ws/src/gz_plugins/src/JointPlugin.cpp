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
#include <optional>


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
    std::vector<float> jointAngles;
    std::vector<float> targetAngles;
    bool new_msg;

  void OnRosMsg(const gz::msgs::Float_V &_msg)
    {
    // Store the received data (assuming it contains 12 angles for simplicity)
      if (_msg.data().size() == 12) {
        for (std::size_t i = 0; i < this->targetAngles.size(); ++i)
        {
          this->targetAngles[i] = _msg.data()[i];
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

    // ROS node
    this->node.Subscribe("/joint_angles", &JointPlugin::OnRosMsg, this);

    }

    virtual void Update 	( 	const gz::sim::UpdateInfo &    _info,
                          gz::sim::EntityComponentManager  & _ecm ) {
      
      if(!this->new_msg){

      }
      else{
        for(int i = 0; i < joints.size(); i++){
          std::vector<double> velocities = {1.0};

          this->joints[i].SetVelocity(_ecm, velocities);
          auto velocity = this->joints[i].Velocity(_ecm);
          
         
          // Print joint name
          auto joint_name = this->joints[i].Name(_ecm);
          if (joint_name.has_value())
          {
            gzdbg << "Joint name: " << joint_name.value() << std::endl;
          }

          // Print current velocity
          auto joint_velocity = this->joints[i].Velocity(_ecm);

          if (joint_velocity.has_value())
          {
            gzdbg << "Current velocity: " << joint_velocity.value().data() << std::endl;
          }

          gzdbg << "The command ran somehow\n";

        } 
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


