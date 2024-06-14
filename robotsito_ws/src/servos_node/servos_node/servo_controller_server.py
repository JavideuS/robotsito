import rclpy
from rclpy.action import ActionServer
from rclpy.node import Node
from custom_action_interfaces.action import ServoLegs

#inherits from Node
class ActionServer(Node): 

    def __init__(self):
        #Initializing Node
        super.__init__("gait_action_server<3")
        #Initialize node to action server and save it in _action_client variable
        #Parameters -? Node, Node_type,Node_name
        self._action_server = ActionServer(
            self,
        )

def main():
    rclpy.init()


if __name__ == "__main__":
    main()