from utils.field import (
    flip_poses,
    update_table,
    NT_Updater,
)
from toolkit.motors.ctre_motors import TalonFX

import config

from toolkit.subsystem import Subsystem

class SingleMotorSubsystem(Subsystem): # single motor class 
    def __init__(self): 
        super().__init__()
        self.motor = TalonFX(config.front_left_move_id, foc=config.foc_active, config=config.MOVE_CONFIG, inverted=config.front_left_move_inverted)
        
    def move_motor(self):
        self.motor.set_voltage(0.1) # set speed of motor to 0.1 volts

    def stop_motor(self):
        self.motor.set_voltage(0)

    def init(self):
        print("Initializing single motor subsystem")
        self.motor.init() #IMPORTANT: HAVE TO INIT ALL TALONFX MOTORS OR ERROR AND DO IT HERE NOT AT __init__
    # order of operations: __init__() called first, then init() called later.

class Robot:
    drivetrain = SingleMotorSubsystem()

class Field:
    # odometry = sensors.FieldOdometry(Robot.drivetrain)
    nt_reporter = NT_Updater("Field")

    @staticmethod
    def flip_poses():
        print("Flipping Pos")
        flip_poses()


    @staticmethod
    def update_field_table(debug=False): #TODO Why doesn't this also call update_odometry? 
        print("Updating Table")
        update_table(Field.nt_reporter, False)

