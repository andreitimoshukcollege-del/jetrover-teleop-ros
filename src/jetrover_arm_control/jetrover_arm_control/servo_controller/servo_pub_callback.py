
from jetrover_arm_control.board_manager.robot_board_manager import BoardManager
from jetrover_arm_control.servo_controller.servo_controller import ServoController
from jetrover_arm_control.servo_controller.servo_id_enum import SERVO_ENUM

class ServoPublisherCallback:

    def __init__(self, boardMgr: BoardManager):
        self.boardMgr = boardMgr
        self.servoController = ServoController(boardMgr= self.boardMgr)

    def getServoPosPulse(self, servoID: SERVO_ENUM):
        pulse = self.servoController.getRawPos(servoID=servoID.value)
        return pulse

    def getServoDeg(self, servoID: SERVO_ENUM):
        deg = self.servoController.getPos(servoID.value)
        return deg

    # def getServoBase(self):
    #     deg = self.servoController.getPos(SERVO_ENUM.BASE_SERVO.value)
    #     return deg
    
    # def getServoLowerArm(self):
    #     deg = self.servoController.getPos(SERVO_ENUM.LOWER_ARM.value)
    #     return deg

    # def getServoMiddleArm(self):
    #     deg = self.servoController.getPos(SERVO_ENUM.MIDDLE_ARM.value)
    #     return deg
    
    # def getServoUpperArm(self):
    #     deg = self.servoController.getPos(SERVO_ENUM.UPPER_ARM.value)
    #     return deg

    # def getServoGripperBase(self):
    #     deg = self.servoController.getPos(SERVO_ENUM.GRIPPER_BASE.value)
    #     return deg
    
    # def getServoGripperMain(self):
    #     deg = self.servoController.getPos(SERVO_ENUM.GRIPPER_MAIN.value)
    #     return deg
        
           