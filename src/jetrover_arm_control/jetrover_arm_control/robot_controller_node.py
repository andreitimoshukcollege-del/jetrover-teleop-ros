import rclpy
from rclpy.node import Node
from std_msgs.msg import Bool, Float32, Int32, String

from datetime import datetime, timezone

from time import sleep

from jetrover_arm_control.board_manager.robot_board_manager import BoardManager
from jetrover_arm_control.robot_data.robot_val import RobotVal
from jetrover_arm_control.servo_controller.servo_sub_callback import ServoSubscriberCallback
from jetrover_arm_control.servo_controller.servo_pub_callback import ServoPublisherCallback
from jetrover_arm_control.servo_controller.servo_controller import ServoController
from jetrover_arm_control.servo_controller.servo_id_enum import SERVO_ENUM


class RobotController(Node):

    def __init__(self):
        super().__init__("robot_controller_node")

        self.board = None
        self.boardMgr = BoardManager()
        self.board = self.boardMgr.getBoard()
        
        self.get_logger().info("Initializing Servo Data Manager...")

        self.robotData = RobotVal(boardMgr = self.boardMgr)
        self.servoSubManager = ServoSubscriberCallback(boardMgr = self.boardMgr)
        self.servoPubManager = ServoPublisherCallback(boardMgr = self.boardMgr)
        self.servoController = ServoController(boardMgr = self.boardMgr)

        # Subscribers
        self.get_logger().info("Initializing Servo Subscribers...")
        self.resetArmSub = self.create_subscription(Float32, "/arm/servo/reset", self.servoSubManager.resetArmCbk, 5)
        
        self.setBaseSub = self.create_subscription(Float32, "/arm/servo/base", self.servoSubManager.setBaseCbk, 1)
        self.setLowerArmSub = self.create_subscription(Float32, "/arm/servo/joint/lower", self.servoSubManager.setJointLowerCbk, 1)
        self.setMiddleArmSub = self.create_subscription(Float32, "/arm/servo/joint/middle", self.servoSubManager.setJointMiddleCbk, 1)
        self.setUpperArmSub = self.create_subscription(Float32, "/arm/servo/joint/upper", self.servoSubManager.setJointUpperCbk, 1)
        self.setGripperBaseSub = self.create_subscription(Float32, "/arm/servo/gripper/base", self.servoSubManager.setGripperBaseCbk, 1)
        self.setGripperMainSub = self.create_subscription(Float32, "/arm/servo/gripper/main", self.servoSubManager.setGripperMainCbk, 1)

        self.buzzerSub = self.create_subscription(Bool, "/robot/buzzer", self.robotData.setBuzzerCbk, 1)
        self.torqueState = self.create_subscription(Int32, "/arm/servo/set/torque", self.servoSubManager.setServoTorqueCbk, 1)

        self.get_logger().info("Servo Subscribers initialized.")

        # Publishers
        self.get_logger().info("Initializing Servo Publishers.")
        self.baseName = "Base Servo"
        self.lowerArmName = "Lower Arm"
        self.middleArmName = "Middle Arm"
        self.upperArmName = "Upper Arm"
        self.gripperBaseName = "Gripper Base"
        self.gripperMainName = "Gripper Main"

        self.batteryName = "Robot Battery"
        self.tempName = "Robot Temp"
        self.voltName = "Robot Volt"
        self.torqueName = "Robot Torque"
        
        self.pollRate = 0.02
        self.stepSize = 50

        self.lastBasePos = None

        self.baseTopic = "/unity/robot/servo/base"
        self.lowerArmTopic = "/unity/robot/servo/joint/lower"
        self.middleArmTopic = "/unity/robot/servo/joint/middle"
        self.upperArmTopic = "/unity/robot/servo/joint/upper"
        self.gripperBaseTopic = "/unity/robot/servo/gripper/base"
        self.gripperMainTopic = "/unity/robot/servo/gripper/main"

        self.servoTempTopic = "/robot/servo/temperature"
        self.servoVoltTopic = "/robot/servo/voltage"
        self.servoTorqueTopic = "/robot/servo/torque"

        self.batteryTopic = "/robot/battery"

        self.basePub = self.createPublisher(self.baseName, Int32, self.baseTopic, self.pollRate, \
                                            self.basePublisher)
        self.lowerArmPub = self.createPublisher(self.lowerArmName, Int32, self.lowerArmTopic, self.pollRate, \
                                            self.lowerArmPublisher)
        #self.middleArmPub = self.createPublisher(self.middleArmName, Int32, self.middleArmTopic, self.pollRate, \
        #                                    self.middleArmPublisher)
        self.upperArmPub = self.createPublisher(self.upperArmName, Int32, self.upperArmTopic, self.pollRate, \
                                            self.upperArmPublisher)
        self.gripperBasePub = self.createPublisher(self.gripperBaseName, Int32, self.gripperBaseTopic, self.pollRate, \
                                            self.gripperBasePublisher) 
        self.gripperMainPub = self.createPublisher(self.gripperMainName, Int32, self.gripperMainTopic, self.pollRate, \
                                            self.gripperMainPublisher) 

        self.batteryPub = self.createPublisher(self.batteryName, Int32, self.batteryTopic, 5, \
                                            self.batteryPublisher)  
        #self.tempPub = self.createPublisher(self.tempName, String, self.servoTempTopic, 5, \
        #                                   self.servoTempPublisher)
        #self.voltPub = self.createPublisher(self.voltName, String, self.servoVoltTopic, 5, \
        #                                   self.servoVoltPublisher)
        #self.torquePub = self.createPublisher(self.torqueName, String, self.servoTorqueTopic, 5, \
        #                                    self.servoTorquePublisher)                                      
        self.get_logger().info("Servo Publishers initialized.")

        self.get_logger().info("Servo Data Manager initialized.")

    
    # --------------------Publishers--------------------------- #
        
    def basePublisher(self):
        data = self.servoController.getRawPos(SERVO_ENUM.BASE_SERVO.value)
        if data is None:
            return
        servoPos = self.servoController.pulseToDeg(data)
        msg = Int32()
        msg.data = servoPos

        self.basePub.publish(msg)
        
        # pulse = self.servoPubManager.getServoPosPulse(SERVO_ENUM.BASE_SERVO)
        
        # if self.lastBasePos is None or abs(pulse - self.lastBasePos) >= 50:
        #     deg = self.servoPubManager.getServoDeg(SERVO_ENUM.BASE_SERVO)
        #     msg = Int32()
        #     msg.data = deg
            
        #     # Publish the message
        #     self.basePub.publish(msg)
            
        #     # Update the last published pulse value
        #     self.lastBasePos = pulse
        #     print("base"+ str(pulse))

        sleep(0.02)

    def lowerArmPublisher(self):
        data = self.servoController.getRawPos(SERVO_ENUM.LOWER_ARM.value)
        if data is None:
            return
        servoPos = self.servoController.pulseToDeg(data)
        msg = Int32()
        msg.data = servoPos


        # deg = self.servoPubManager.getServoLowerArm()
        # msg = Int32()
        # msg.data = deg
    
        self.lowerArmPub.publish(msg)

        sleep(0.02)

    def middleArmPublisher(self):
        data = self.servoController.getRawPos(SERVO_ENUM.MIDDLE_ARM.value)
        if data is None:
            return
        servoPos = self.servoController.pulseToDeg(data)
        msg = Int32()
        msg.data = servoPos

        # deg = self.servoPubManager.getServoMiddleArm()
        # msg = Int32()
        # msg.data = deg
    
        self.middleArmPub.publish(msg)

        sleep(0.02)

    def upperArmPublisher(self):
        data = self.servoController.getRawPos(SERVO_ENUM.UPPER_ARM.value)
        if data is None:
            return
        servoPos = self.servoController.pulseToDeg(data)
        msg = Int32()
        msg.data = servoPos

        # deg = self.servoPubManager.getServoUpperArm()
        # msg = Int32()
        # msg.data = deg
    
        self.upperArmPub.publish(msg)

        sleep(0.02)

    def gripperBasePublisher(self):
        data = self.servoController.getRawPos(SERVO_ENUM.GRIPPER_BASE.value)
        if data is None:
            return
        servoPos = self.servoController.pulseToDeg(data)
        msg = Int32()
        msg.data = servoPos

        # deg = self.servoPubManager.getServoGripperBase()
        # msg = Int32()
        # msg.data = deg
    
        self.gripperBasePub.publish(msg)

        sleep(0.02)

    def gripperMainPublisher(self):
        data = self.servoController.getRawPos(SERVO_ENUM.GRIPPER_MAIN.value)
        if data is None:
            return
        servoPos = self.servoController.pulseToDeg(data)
        msg = Int32()
        msg.data = servoPos

        # deg = self.servoPubManager.getServoGripperMain()
        # msg = Int32()
        # msg.data = deg

        self.gripperMainPub.publish(msg)

        sleep(0.02)

    def batteryPublisher(self):
        data = self.robotData.getBattery()
        if data is None:
            return
        msg = Int32()
        msg.data = data

        self.batteryPub.publish(msg)

        sleep(0.02)

    def servoTempPublisher(self):
        data = self.servoController.getServoTemp()
        if data is None:
            return
        msg = String()
        msg.data = data

        self.tempPub.publish(msg)

        sleep(0.02)

    def servoVoltPublisher(self):
        data = self.servoController.getServoVolt()
        if data is None:
            return
        msg = String()
        msg.data = data

        self.voltPub.publish(msg)

        sleep(0.02)

    def servoTorquePublisher(self):
        data = self.servoController.getServoTorque()
        if data is None:
            return
        msg = String()
        msg.data = data

        self.torquePub.publish(msg)

        sleep(0.02)

    def createPublisher(self, publisherName, msgType, msgTopic, pollRate: int, clbFunc, queueSize=1):
        dataPub = self.create_publisher(msgType, msgTopic, queueSize)
        self.get_logger().info(f"Publisher - {publisherName} Created")
        self.timer = self.create_timer(pollRate, clbFunc) 

        return dataPub

def main():
    rclpy.init()
    robot_controller = RobotController()

    try:
        rclpy.spin(robot_controller)
    except KeyboardInterrupt:
        robot_controller.get_logger().info("Node interrupted by user keyboard")
    finally:
        robot_controller.get_logger().info("Shutting Down ROS Subscriber Node")
        robot_controller.destroy_node()
        rclpy.shutdown()

if __name__ == "__main__":
    main()
    
