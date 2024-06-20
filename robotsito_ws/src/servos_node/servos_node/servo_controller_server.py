import time
import rclpy
from rclpy.action import ActionServer, CancelResponse, GoalResponse
from rclpy.node import Node
from custom_action_interfaces.action import ServoLegs
#Change the python interpreter to the one of the venv to use the library correctly
from adafruit_servokit import ServoKit


#inherits from Node
class ActionServer(Node): 

    #Constructor
    def __init__(self):
        #Initializing Node
        super().__init__("gait_action_server")
        #Then we initialize the board ant the servos
        #Even though it only uses 12 channels you need to initilize the servokit for the 16 channels
        SERVO_CHANNELS=16
        self.kit = ServoKit(channels=SERVO_CHANNELS)
        #Initialize pulse width
        for i in range(SERVO_CHANNELS):
            #Min pulse -> 500
            #Max pulse -> 2500
            self.kit.servo[i].set_pulse_width_range(500,2500)
            #By default every servo will be initialized to 90 degrees
            self.kit.servo[i].angle(90)
            time.sleep(0.5)
        
        self.status = [90] * 12

        #Initialize node to action server and save it in _action_client variable
        #Parameters -? Node, Node_type,Node_name,action_callback
        self._action_server = ActionServer(
            self,
            ServoLegs,
            "ServoLegs",
            self.execute_callback
        )

        #Lastly print a message indicating that the action server was initialized
        self.get_logger.info("Gait control action server has been initialized succesfuly <3")

    #Destructor    
    def destroy(self):
        self._action_server.destroy()
        super().destroy_node()


    def goal_callback(self, goal_request):
        """Accept or reject a client request to begin an action."""
        # This server allows multiple goals in parallel
        self.get_logger().info('Received goal request')
        return GoalResponse.ACCEPT

    def cancel_callback(self, goal_handle):
        """Accept or reject a client request to cancel an action."""
        self.get_logger().info('Received cancel request')
        return CancelResponse.ACCEPT

    #The execute_callback is the function that the function will do when called 
    def execute_callback(self,goal_handle):
        """Parameters:
        self -> the node 
        goal_handle -> The array with 12 angle postions    
            
            """    
        #First we call the node logger and print the starting/call of the function
        self.get_logger.info("Executing goal...")

        #First we retrieve the angles passed
        angles = goal_handle.request.angles
        #If the angles !=12 position -> Warning
        if(len(angles) != 12):
            goal_handle.abort()
            return ServoLegs.Result(success=False)

        #Then we prepare the feedback_msg
        feedback_msg = ServoLegs.Feedback()
        #Feedback is the current status of the servos
        feedback_msg.current_angles = self.status
        for i in range(12):
            if goal_handle.is_cancel_requested:
                goal_handle.canceled()
                self.get_logger().info('Goal canceled')
                return ServoLegs.Result(success=False)
            
            self.kit.servo[i].angle = angles[i]
            #We actualize the status
            self.status[i] = angles[i]
            
            # Provide feedback
            feedback_msg.current_angles = self.current_angles
            goal_handle.publish_feedback(feedback_msg)
            time.sleep(0.05)  # Adjust the sleep duration as needed
        
        #Goal succed
        goal_handle.succeed()
        
        return ServoLegs.Result(success=True)


def main(args=None):
    rclpy.init(args=args)
    #Initializing the node
    node = ActionServer()
    #Calling the node
    rclpy.spin(node)
    #Destroying the node
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()