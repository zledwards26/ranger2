import math

#linear algebra
import numpy as np

#ROS2
import rclpy
from rclpy.node import Node

#ZedX Objects
from zed_msgs.msg import ObjectsStamped

#message type for camera position
from geometry_msgs.msg import PoseStamped, Point, Quaternion, PointStamped

from std_msgs.msg import Float32


class lookForGateNode(Node):
    def __init__(self):
        print("opening gate finder")
        
        super().__init__('look_for_gate_node')
        
        #start a timer when you see a gate.
        self.gateTimer = 0

        #When you see a gate for long enough, call self.iFoundAGate
        self.gateTimerSuccess = 10

        #subscribe to detections 
        self.create_subscription(ObjectsStamped, '/zed/zed_node/obj_det/objects', self.lookForGate,10)

        #Subscribe rotations and translations from global space to camera-relative space
        self.create_subscription(PoseStamped, '/zed/zed_node/pose', self.updatePos,10)

        self.camera_position = Point()
        self.camera_orientation = Quaternion()

        #initiate channel for gates
        self.gatePosPublisher = self.create_publisher(PointStamped, '/gatePos', 10)

        #channel for single gateposts
        self.gatepostPosPublisher = self.create_publisher(PointStamped, '/gatepostPos', 10)

        #if it sees just one gatepost, tell gotogate the angle to rotate and center the gatepost
        self.gatepostThetaPublisher = self.create_publisher(Float32, '/gatepostThetaXXXX', 10)

    def updatePos(self, msg):
        self.camera_position = msg.pose.position
        self.camera_orientation = msg.pose.orientation
        #print(self.camera_orientation)
        #print(self.camera_position)



    def iFoundAGate(self,gatepostPoses):
        pos0 = gatepostPoses[0]
        pos1 = gatepostPoses[1]

        #average poses of gatepots to cet position of gate center
        x = float((pos0[0] + pos1[0]) / 2)
        y = float((pos0[1] + pos1[1]) / 2)
        z = float((pos0[2] + pos1[2]) / 2)

        

        print("I found a gate!! It's coordinates are x:", x, "y:", y, "z:", z)

        

        #convert it to be in the map frame so goToGate can use it
        gatePos = self.convertToMap(x, y, z)

        self.gatePosPublisher.publish(gatePos)


    def lookForGate(self,msg):
        print("looking for gate")
        #print("Message: ")

        if len(msg.objects) == 0:
            print("No objects found")
            return
        
        #count gates
        
        
        gatepostPoses = []
        for object in msg.objects:
            
            print("found a ", object.label, "! I'ts label id is ", object.label_id, "!")
            
            if object.label != "GatePost":
                continue
            """working on a system to publish gate positions
            gatepostPos = PointStamped()
            gatepost.header = msg.header
            
            x = position[0]
            y = position[1]
            z = position[2]"""
            

            gatepostPoses.append(object.position)

        if len(gatepostPoses) == 2:
            print("found two posts!! (:")

            #start the gate timer. when it sees a gate for long enough, call self.iFoundAGate
            self.gateTimer +=1
            print("gate timer at", str(self.gateTimer))

            if self.gateTimer == self.gateTimerSuccess:
                self.iFoundAGate(gatepostPoses)
                self.gateTimer = 0

        elif len(gatepostPoses) == 1:
            print("found one post")

        elif len(gatepostPoses) == 3:
            print("found three posts?! SOMETHING IS VERY WRONG")



        #did not find exactly two gateposts. reset gate timer
        if len(gatepostPoses) != 2:
            self.gateTimer = 0

        #if it found any number of gateposts, tell gotogate the angle needed to center those gateposts
        if len(gatepostPoses) == 0:
            return

        #average gatepost pos
        x = 0.0
        y = 0.0
        z = 0.0
        for post in gatepostPoses:
            x += post[0]
            y += post[1]
            z += post[2]

        l = len(gatepostPoses)
        
        x /= l
        y /= l
        z /= l

        gatepostPos = self.convertToMap(x, y, z)
        
        self.gatepostPosPublisher.publish(gatepostPos)
        
        #convert to angle
        theta = Float32()
        theta.data = math.atan2(x, z)

        self.gatepostThetaPublisher.publish(theta)
        



        
    def convertToMap(self, x, y, z):
        gatePos = np.array([x,y,z])

        R = createRotationMatrix(self.camera_orientation)

        collumPos = np.atleast_2d(gatePos).T
        
        collumPos = np.matmul(R,collumPos)

        gatePos = collumPos.T[0]
        
        cameraPos = self.camera_position
        
        cx = cameraPos.x
        cy = cameraPos.y
        cz = cameraPos.z

        cameraPos = np.array([cx,cy,cz])

        gatePos = np.add(gatePos, cameraPos)

        x = gatePos[0]
        y = gatePos[1]
        z = gatePos[2]

        gatePos = PointStamped()

        # Fill header
        gatePos.header.stamp = self.get_clock().now().to_msg()
        gatePos.header.frame_id = "map"  # replace with actual frame from your TF tree

        #set position
        gatePos.point.x = x
        gatePos.point.y = y
        gatePos.point.z = z

        return gatePos
    
    def destroy_node(self):
        print("closing gate looker for (look for gater? Looker for gates?)")
        
        super().destroy_node()


#to be used if we need to transform the gate position from one coordinate frame to another
def createRotationMatrix(rotation):
    #take a rotation quaternion and a translation vector and return a 4x4 transformation matrix.

    #note: a 3x3 matrix can only rotate, but a 4x4 matrix can rotate and translate.

    
    qx = rotation.x
    qy = rotation.y
    qz = rotation.z
    qw = rotation.w


    
    # Normalize quaternion
    norm = math.sqrt(qx*qx + qy*qy + qz*qz + qw*qw)
    if norm == 0:
        raise ValueError("Invalid quaternion with zero norm")

    qx /= norm
    qy /= norm
    qz /= norm
    qw /= norm

    #This link explains converting a quaternion to a rotation matrix: https://automaticaddison.com/how-to-convert-a-quaternion-to-a-rotation-matrix/
    
    # First row of the rotation matrix
    r00 = 2 * (qw * qw + qx * qx) - 1
    r01 = 2 * (qx * qy - qw * qz)
    r02 = 2 * (qx * qz + qw * qy)

    # Second row of the rotation matrix
    r10 = 2 * (qx * qy + qw * qz)
    r11 = 2 * (qw * qw + qy * qy) - 1
    r12 = 2 * (qy * qz - qw * qx)

    # Third row of the rotation matrix
    r20 = 2 * (qx * qz - qw * qy)
    r21 = 2 * (qy * qz + qw * qx)
    r22 = 2 * (qw * qw + qz * qz) - 1

    R = np.array([
        [r00, r01, r02],
        [r10, r11, r12],
        [r20, r21, r22],
    ])

    return R



def main(args=None):
    rclpy.init(args=args)

    node = lookForGateNode()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass

    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
