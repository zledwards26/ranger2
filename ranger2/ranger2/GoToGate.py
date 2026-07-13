#ROS2
import rclpy
from rclpy.node import Node

from std_msgs.msg import Float32

#message type for camera position
from geometry_msgs.msg import PoseStamped, PointStamped, Twist

from math import sqrt, atan2, pi

from copy import deepcopy

from mavros_msgs.msg import State

#Numpy
import numpy as np
import quaternion

from random import random

class goToGateNode(Node):
    def __init__(self):
        print("opening gate goer to")
        
        super().__init__('go_to_gate_node')
        
        self.state = 0
        """
        State key:
        -1: ROV mode                    entered from any state when operator input is detected, exited my operator override
        0: inactive                     starting state, go to 1 when it detects the cube is armed
        1: descend to desired depth     go to 2 when descent is complete
        2: spin                         to 3 when a gatepost object is detected 
        3: center detected gatePost,    to 4 when look_for_gate publishes a gate, to 2 when no gate is detected
        4: travel to gate               
        
        To override state, use
        ros2 topic pub --once /SetState std_msgs/msg/Float32 "{data: 1.0}"

        to build package, use
        colcon build --packages-select ranger2
        
        """

        """
        self.state determines what state the robot is in.

        Each state will have callbacks on ROS topics that will cause the robot to enter that state from specified other states.
        Entering a state may also call a method to move the sub.


        The sub will be moved by controling self.waypoint (a PoseStamped() data type). self.waypoint is periodically published to MavROS
        No callback on a ROS topic will directly read or set self.waypoint. Instead, this will be handeled by a set of methods to move the sub around.
        Seperating State logic and movement will hopefully let us debug the two seperately.

        """

        #Configure me plz :3
        self.waypointPeriod = 0.10 #time between sending waypoints

        self.targetDepth = 0.2# -0.5
        self.positionError = 0.10


        #containers for position and waypoint
        self.pos = PoseStamped()

        self.waypoint = PoseStamped()
        self.waypoint.header.frame_id = "map"

       
        # ------- PUBLISHERS -------
        #publish state
        self.statePublisher = self.create_publisher(Float32, '/state', 10)

        #periodically publish waypoints
        self.waypointPublisher = self.create_publisher(PoseStamped, '/mavros/setpoint_position/local',10)

         # ------- CALLBACKS -------

        self.create_timer(self.waypointPeriod, self.sendWaypoint)

        #override state when the ROV operator publishes to /setState
        self.create_subscription(Float32, '/SetState', self.setState, 10)

        #use turtle_teleop_key to drive sub
        self.create_subscription(Twist, '/turtle1/cmd_vel', self.turtleTeleopCallback, 10)

        #use rviz2 clicked point to drive sub
        self.create_subscription(PointStamped, '/clicked_point', self.clickedPointCallback, 10)

        #When gate detecter is sure about a gate, it will tell the sub to go there.
        self.create_subscription(PointStamped, '/gatePos', self.gateCallback, 10)

        #When the gate detector sees just one gate, point at it.
        self.create_subscription(PointStamped, '/gatepostPos', self.gatepostCallback, 10)

        #callback to update sub position
        self.create_subscription(PoseStamped, '/mavros/local_position/pose', self.posCallback,10)

        #Use zed position instead of mavros for debuging
        self.create_subscription(PoseStamped, '/zed/zed_node/pose', self.posCallback,10)

        #callback to test when sub is armed and start run
        self.create_subscription(State, '/mavros/state', self.armingCallback, 10)
        

    """*****************Callbacks  ***********************"""
    #state override
    def setState(self,msg):
        self.state = int(msg.data)

        print("state:", self.state)


        #some states need to do something when you enter them:

        #enter state 1
        if self.state == 1:
            
            self.startDescend()
    

        

    # -------STATE -1 ROV MODE: -------

    #callback to control the sub with rviz clicked point
    def clickedPointCallback(self,msg):
        point = msg.point

        self.state = -1
        self.goToPoint(point.x, point.y, self.targetDepth)
        #make the waypoint quaternion right side up
        self.correctWaypointQuaternion()

    #callback to control sub with turtle teleop
    def turtleTeleopCallback(self,msg):
        self.state = -1

        theta = msg.angular.z * (pi / 6) / 2

        if theta != 0: #rotation command
            self.rotateWaypoint(theta)
            self.correctWaypointQuaternion()
            return

        self.moveWaypointForward(msg.linear.x / 10) #move forward command
    
    # -------STATE 1 DESCEND: -------

    #callback on /mavros/sate arms the sub
    def armingCallback(self, msg):

        #enter state 1
        if msg.armed and self.state == 0:
            self.state = 1
            self.startDescend()

    # -------STATE 2 SPIN: -------

    #callback for when a waypoint is reached
    def reachWaypointCallback(self):

        self.correctWaypointQuaternion()

        #enter state 2
        if self.state == 1:
            self.state = 2
    
    #spin is called periodically
    def spin(self):
        self.rotateWaypoint(random() * 2.0 * pi)

    # -------STATE 3 CENTER THE GATE: -------

    def gatepostCallback(self,msg):
        if self.state != 2:
            return
        
        self.pointAtPoint(msg.point.x, msg.point.y, self.targetDepth)

    # -------STATE 4 TRAVEL TO GATE: -------

    #callback on gate detections
    def gateCallback(self,msg):
        
        #enter state 4 
        #If you see a gate when you are looking for a gate, go to the gate
        if self.state == 3 or self.state == 2 or self.state == 1:
            self.goToPoint(msg.point.x, msg.point.y, self.targetDepth)
            self.correctWaypointQuaternion()
            self.state = 4
    
    # -------ALL STATES: -------

    #update AUV position
    def posCallback(self,msg):
        #print("setting self.pos")
        self.pos = msg

        if self.state == 0:
            self.waypoint = self.pos

    #periodically send waypoints,  test if waypoints are reached, and publish information on state
    def sendWaypoint(self):
        self.waypoint.header.stamp = self.get_clock().now().to_msg()
        
        self.waypointPublisher.publish(self.waypoint)

        #Test if the current waypoint has been reached
        goal = self.waypoint.pose.position
        pos = self.pos.pose.position

        dx = goal.x - pos.x
        dy = goal.y - pos.y
        dz = goal.z - pos.z

        distance = sqrt(dx * dx + dy * dy + dz * dz)

        #reach waypoint callback for spin state
        if distance < self.positionError:
            self.reachWaypointCallback()


        #publish state
        state = Float32()
        state.data = float(self.state)
        self.statePublisher.publish(state)
        
        if self.state == 2 and random() > 0.5:
            self.spin()
        
        #exit state 3 when you no longer see gates
        #(this will not interrupt rotating to point at the gate and hoefully it will see the gate again before it rotates to a random angle)
        if self.state == 3:
            self.state == 2

    """*****************Methods to move the waypoint (moving it forward, rotating it)***********************"""

    #method to remove pitch and roll from waypoint orientation. This keeps the sub right side up.
    def correctWaypointQuaternion(self):
        #yaw from quaternion with vertical z-axis: https://robotics.stackexchange.com/questions/16471/get-yaw-from-quaternion

        #quaternion from euler angles: https://en.wikipedia.org/wiki/Conversion_between_quaternions_and_Euler_angles

        q = self.waypoint.pose.orientation
        yaw = atan2(2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y * q.y + q.z * q.z))

        self.waypoint.pose.orientation.w = np.cos(yaw/2)
        self.waypoint.pose.orientation.x = 0.0
        self.waypoint.pose.orientation.y = 0.0
        self.waypoint.pose.orientation.z = np.sin(yaw/2)


    def pointAtWaypoint(self):

        #point the sub at it's waypoint
        pos2 = self.waypoint.pose.position

        x = pos2.x
        y = pos2.y
        z = pos2.z

        self.pointAtPoint(x, y, z)

    def pointAtPoint(self, x, y, z):
        #change the waypoint rotation quaternion so that the sub is pointed forward.

        #to do this, find the displacement vector to the target position, then find the quaternion that rotates +i to this displacement vector.
        #This quaternion represents the rotation that will point the sub at that point
        #This site was used for the formula for the rotation: https://raw.org/proof/quaternion-from-two-vectors/

        pos1 = self.pos.pose.position

        

        dx = x - pos1.x
        dy = y - pos1.y
        dz = z - pos1.z

        #To normalize displacement vector
        mag = sqrt(dx * dx + dy * dy + dz * dz)

        
        if mag < 1e-6:
            return


        dx /= mag
        dy /= mag
        dz /= mag


        dot = dx

        #math breaks down for some vectors
        if dot < -0.9999:
            self.waypoint.pose.orientation.w = 0.0
            self.waypoint.pose.orientation.x = 0.0
            self.waypoint.pose.orientation.y = 1.0
            self.waypoint.pose.orientation.z = 0.0
            return


        cross = [0, -dz, dy]

        #to normalize quaternion
        mag = sqrt((1 + dot) ** 2 + dz ** 2 + dy ** 2)

        if mag < 1e-6:
            return

        self.waypoint.pose.orientation.w = (1 + dot) / mag
        self.waypoint.pose.orientation.x = 0.0
        self.waypoint.pose.orientation.y = -dz / mag
        self.waypoint.pose.orientation.z = dy / mag

        

    #rotate the waypoint position by an angle
    def rotateWaypoint(self,theta):

        p = self.pos.pose.orientation

        quat0 = np.quaternion(p.w, p.x, p.y, p.z)

        quat1 = np.quaternion(np.cos(theta/2), 0.0, 0.0, np.sin(theta/2))

        quat2 = quat0 * quat1

        self.waypoint.pose.orientation.w = quat2.w
        self.waypoint.pose.orientation.x = quat2.x
        self.waypoint.pose.orientation.y = quat2.y
        self.waypoint.pose.orientation.z = quat2.z
    
    def moveWaypointForward(self, d):

        #to rotate a vector by a quaternion, you add a scalar to the vector to get a quaterion representing position. Yes that is correct.
        #you then multiply rotation quaterion times position quaternion times conjugate of rotation quaternion.
        #the result of this multiplication is the position quaternion rotated by the rotation quaternion.

        #to move the sub forward, we start with a 1.0i vector, rotate it by the sub's rotation to get a vector pointing forward
        #this vector is the nadded to the sub's position to get a new waypoint
        quat0 = np.quaternion(0.0, 1.0, 0.0, 0.0)
        
        q = self.waypoint.pose.orientation

        quat1 = np.quaternion(q.w, q.x, q.y, q.z)

        quat2 = quat1 * quat0 * quat1.conjugate()

        self.waypoint.pose.position.x = self.pos.pose.position.x + quat2.x * d
        self.waypoint.pose.position.y = self.pos.pose.position.y + quat2.y * d

    def goToPoint(self, x, y, z):
        self.waypoint = deepcopy(self.pos)
        self.waypoint.pose.position.x = x
        self.waypoint.pose.position.y = y
        self.waypoint.pose.position.z = z

        self.pointAtWaypoint()

    def startDescend(self):
        

        print(self.pos)
        self.waypoint = deepcopy(self.pos)
        print(self.waypoint)

        self.waypoint.pose.position.z = self.targetDepth

        self.correctWaypointQuaternion()


    
    """***************** Destroy node ***********************"""
    def destroy_node(self):
        print("me no go to gate. :c ")
    
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)

    node = goToGateNode()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass

    node.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()