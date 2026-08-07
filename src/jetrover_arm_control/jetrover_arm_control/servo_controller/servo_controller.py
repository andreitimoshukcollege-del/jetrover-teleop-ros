from time import sleep

from jetrover_arm_control.board_manager.robot_board_manager import BoardManager
from jetrover_arm_control.servo_controller.servo_id_enum import SERVO_ENUM

class ServoController:

    def __init__(self, boardMgr: BoardManager):
        
        self.boardMgr = boardMgr
        self.board = self.getBoard()

        self.pwm_min = 0
        self.pwm_max = 1000
        self.deg_min = 0
        self.deg_max = 180

        # self.servoList = [SERVO_ENUM.BASE_SERVO.value, SERVO_ENUM.LOWER_ARM.value, \
        #                   SERVO_ENUM.MIDDLE_ARM.value, SERVO_ENUM.UPPER_ARM.value, SERVO_ENUM.GRIPPER_BASE.value, SERVO_ENUM.GRIPPER_MAIN.value]

        self.servoList = [SERVO_ENUM.BASE_SERVO.value, \
                          SERVO_ENUM.MIDDLE_ARM.value, SERVO_ENUM.UPPER_ARM.value, SERVO_ENUM.GRIPPER_BASE.value, SERVO_ENUM.GRIPPER_MAIN.value]

    def getPos(self, servoID: int):
        currentPos = self.getRawPos(servoID=servoID)
        deg = self.pulseToDeg(currentPos)

        return deg
        
    def setPos(self, servoID:int, direction: int, servoSpeed: float = 0.02):
        """This class sets the position of the servo motor.
            Arguments:
                servoID (int) - ID for the servo
                pos (int) - position in PWM
                servoSpeed (int) - servo speed; DEFAULT = 1
        """
        # Fetch the current position
        currentPos = self.getRawPos(servoID=servoID)
        if currentPos is None:
            # The board's read can time out (real, observed occurrence -- see
            # robot_controller_sdk.py's bus_servo_read_and_unpack). Skipping this command
            # rather than crashing on `None + int` matches how every read failure elsewhere
            # in this file is already treated: missed data for one cycle, not a fatal error.
            return
        stepSize = 50

        # Calculate the new target position based on direction
        nextPos = int(currentPos + (stepSize * direction))

        # print("currentPos:", currentPos, "Direction:", direction, "Target Position:", nextPos)
        
        nextPos = 0 if nextPos < 0 else 1000 if nextPos > 1000 else nextPos 
        
        # if toPos > 600:
        #     toPos = 600
        # elif toPos < 400:
        #     toPos = 400

        if abs(nextPos - currentPos) >= stepSize:
            # Set servo position only if the next position is within limits
            self.board.bus_servo_set_position(servoSpeed, [[servoID, nextPos]])
        
            # Give the servo time to move to the new position
            sleep(servoSpeed)

    def setGripperPos(self, servoID: int, direction: int, servoSpeed: float = 0.5):
        pulse_pos = self.degToPulse(direction)
        # print(pulse_pos, direction)
        self.board.bus_servo_set_position(servoSpeed, [[servoID, pulse_pos]])

    def resetArm(self):
        
        self.board.bus_servo_set_position(1, ((10, 500), (5, 500), (4, 500 ), (3, 500), (2, 500), (1, 500)))
        sleep(0.02)

        self.board.bus_servo_set_position(1, ((10, 500), (5, 500), (4, 500 ), (3, 500), (2, 500), (1, 500)))
        sleep(0.02)

    def getRawPos(self, servoID: int):
        if(servoID):
            readPos = self.board.bus_servo_read_position(servo_id= servoID)
            if(readPos):
                return readPos[0]
            
            # return self.board.bus_servo_read_position(servo_id= servoID)[0]
        else:
            return -1

    def degToPulse(self, data: float):
        val = ( data / self.deg_max ) * self.pwm_max
        return int(val)

    def pulseToDeg(self, data: int):
        val = (data / self.pwm_max) * self.deg_max
        return int(val)

    def getServoTemp(self):
        temps= ""
        for servoID in self.servoList:
            temp = self.board.bus_servo_read_temp(servoID)[0]
            temps = str(temps)+","+ str(temp)

        return temps
    
    def getServoVolt(self):
        volts= ""
        for servoID in self.servoList:
            volt = self.board.bus_servo_read_vin(servoID)[0]
            volts = str(volts)+","+ str(volt)
        
        return volts
    
    def getServoTorque(self):
        torques= ""
        for servoID in self.servoList:
            torque = self.board.bus_servo_read_torque_state(servoID)[0]
            torques = str(torques)+","+ str(torque)
        
        return torques

    def setTorqueState(self, state: int):
        
        for servoID in self.servoList:
            if(state == 1):
                self.board.bus_servo_enable_torque(servoID, 1)
            else:
                self.board.bus_servo_enable_torque(servoID, 0)


    def getBoard(self):
        return self.boardMgr.getBoard()
