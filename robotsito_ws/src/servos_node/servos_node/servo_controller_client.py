import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node

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
            
            )
    print("hola")



def main():
    rclpy.init()


if __name__ == "__main__":
    main()