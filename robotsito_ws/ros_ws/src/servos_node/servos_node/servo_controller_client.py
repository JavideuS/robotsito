import rclpy
from action_msgs.msg import GoalStatus
from rclpy.action import ActionClient
from rclpy.node import Node
from custom_action_interfaces.action import ServoLegs
from std_msgs.msg import UInt8MultiArray, MultiArrayLayout, MultiArrayDimension

#Action class/object
class actionClient(Node):
    
    #Constructor
    def __init__(self):
        #Initialize the node
        super().__init__("gait_action_client")
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
    def send_goal (self,layout,angles,iterations):

        #Initilizing the goal message
        goal_msg = ServoLegs.Goal()
        #Assigning the angles to the goal
        goal_msg.angles = angles
        #Assigning layout
        goal_msg.layout = layout #MultiArrayLayout

        goal_msg.iterations = iterations
        
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

    def walking_pattern(self,iterations):
        
        angles = [
            #Upper leg angles are inverted for left and right, the rest are the same
            [[120, 75, 90], [90, 75, 90], [90, 90, 90] ],
            [[90, 90, 90], [120, 105, 90], [90, 90, 90] ],
            [[90, 90, 90], [120, 75, 90], [90, 90, 90] ],
            [[120, 105, 90], [90, 105, 90], [90, 90, 90] ],
            [[120, 75, 90], [90, 75, 90], [90, 90, 90] ],
            [[90, 90, 90], [120, 105, 90], [90, 90, 90] ]
            
            ];

        #Initializing layout
        layout = MultiArrayLayout()
        
        rows = len(angles) #6 legs
        cols = len(angles[0]) if rows > 0 else 0 
        depth = len(angles[0][0]) if cols > 0 else 0


       #3d array
        
        # First dimension: Rows (legs)
        dim1 = MultiArrayDimension()
        dim1.label = "rows" 
        dim1.size = rows
        dim1.stride = rows * cols * depth  # Total number of elements in the array

        # Second dimension: Columns (phases per leg)
        dim2 = MultiArrayDimension()
        dim2.label = "columns"
        dim2.size = cols
        dim2.stride = cols * depth  # Number of elements in one row (one leg)

        # Third dimension: Depth (joints per phase)
        dim3 = MultiArrayDimension()
        dim3.label = "depth"
        dim3.size = depth
        dim3.stride = depth  # Number of elements in one column (one phase)

        
        layout.dim = [dim1, dim2, dim3]
        layout.data_offset = 0

        # Flatten the 3D array into a 1D list
        flattened_data = [angle for row in angles for phase in row for angle in phase]

        self.send_goal(layout,flattened_data,iterations)
   


def main(args=None):
    rclpy.init(args=args)
    #Initializing action client
    action_client = actionClient()
    
    #Calling walk_pattern
    action_client.walking_pattern(5)

    #Calling the client node
    rclpy.spin(action_client)

    #Destroying the client node
    action_client.destroy_node()
    rclpy.shutdown()





if __name__ == "__main__":
    main()