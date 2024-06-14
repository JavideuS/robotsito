import rclpy
from action_msgs.msg import GoalStatus
from rclpy.action import ActionClient
from rclpy.node import Node
from custom_action_interfaces.action import ServoLegs
#Change the python interpreter to the one of the venv to use the library correctly
from adafruit_servokit import ServoKit
#Action class/object
class actionClient(Node):
    
    #Constructor
    def __init__(self):
        #Initialize the node
        super().__init__("gait_action_client<3")
        #Initialize the node to an actiion cliente
        #Parameters to initialize action client -> Node, Node_type, Node_Name
        self._action_client = ActionClient(
            self,
            ServoLegs,
            "ServoLegs"
            )
        #Message indicating the client was initialized
        self.get_logger().info('Gait control client has been started.<3')

     #To send the goal
    def send_goal (self,angles):
        #Initilizing the goal message
        goal_msg = ServoLegs.Goal()
        #Assigning the angles to the goal
        goal_msg.angles = angles

        #Waiting for the server
        self._action_client.wait_for_server()
        
        #Calling the server
        return self._action_client.send_goal_async(goal_msg)

    #To know the current angle positions
    def feedback_callback(self, feedback_msg):
        self.get_logger().info(f'Current angles: {feedback_msg.current_angles}')


    def goal_response_callback(self, future):
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().info('Goal rejected :(')
            return

        self.get_logger().info('Goal accepted :)')

        self._get_result_future = goal_handle.get_result_async()
        self._get_result_future.add_done_callback(self.get_result_callback)

    def get_result_callback(self, future):
        result = future.result().result
        status = future.result().status
        #Checking the status of the goal
        if status == GoalStatus.STATUS_SUCCEEDED:
            #Printing current angles as result
            self.get_logger().info('Goal succeeded! Result: {0}'.format(result.current_angles))
        else:
            self.get_logger().info('Goal failed with status: {0}'.format(status))

        # Shutdown after receiving a result
        rclpy.shutdown()        
   


def main(args=None):
    rclpy.init(args=args)
    #Initializing action client
    action_client = actionClient()
    #Example angles
    angles = [90, 45, 45, 90, 45, 45, 90, 45, 45, 90, 45, 45]
    #Sending the goal
    actionClient.send_goal(angles)

    #Calling the client node
    rclpy.spin(action_client)
    #Destroying the client node
    action_client.destroy_node()
    rclpy.shutdown()





if __name__ == "__main__":
    main()