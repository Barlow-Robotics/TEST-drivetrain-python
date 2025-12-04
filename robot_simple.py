# import standard Python libraries

# import key robotics libraries
import commands2
import ntcore
import phoenix6 as ctre
import wpilib

# import our libraries
# import command
import config

# import the variable Subsytem from the toolkit library
from oi.keymap import Keymap
from toolkit.subsystem import Subsystem

# import constants
from robot_systems_simple import (  
    Robot,
    Field
)

# import the variables DriverStation and SenableChooser from wpilib for ease of use
from wpilib import DriverStation
import sys

# import the variables
import utils
# from oi.OI import OI VERY PROBLEMATIC ERROR B/C OF STATIC CLASSES

# defining the class "_Robot"
class _Robot(wpilib.TimedRobot):
    def __init__(self):
        super().__init__(period=0.02) # COULD TEST dif periods. 

        # establish variables to access across the class
        self.log = utils.LocalLogger("Robot")
        self.nt = ntcore.NetworkTableInstance.getDefault()
        self.scheduler = commands2.CommandScheduler.getInstance()
        self.color = DriverStation.Alliance.kRed

    # runs on startup
    def robotInit(self):
        # start the log
        self.log._robot_log_setup()
        
        # Initialize Operator Interface
        if config.DEBUG_MODE:
            self.log.setup("WARNING: DEBUG MODE IS ENABLED")
        
        # establish the refresh period for the scheduler
        period = config.period
        self.scheduler.setPeriod(period)
        self.log.info(f"Scheduler period set to {period} seconds")

        # flips the field poses from blue to red alliance field poses
        Field.flip_poses()

        # Initialize subsystems
        def init_subsystems():
            subsystems: list[Subsystem] = list(
                {
                    k: v
                    for k, v in Robot.__dict__.items()
                    if isinstance(v, Subsystem) and hasattr(v, "init")
                }.values()
            )
            # sensors: list = list(
            #     {k: v for k, v in Sensors.__dict__.items()
            #       if isinstance(v, sensors.Sensor) and hasattr(v, 'init')}.values()
            # )
            
            # iterate through each subsystem
            for subsystem in subsystems:
                subsystem.init()

            # for sensor in sensors:
            #     sensor.init()

        # attempt to run the subsystem initialization and log any errors
        try:
            init_subsystems()
            # pass
        except Exception as e:
            self.log.error(e)
            self.nt.getTable("errors").putString("subsystem init", str(e))
            raise e

        # reducing noise from the CAN bus (unused can-bus traffic optimization)
        ctre.hardware.ParentDevice.optimize_bus_utilization_for_all()
        self.robot = Robot()
        Field.update_field_table()

        # log that we have properly initialized the robot
        self.log.complete("Robot initialized")

    # run cycle for every period as defined
    def robotPeriodic(self):
        table = ntcore.NetworkTableInstance.getDefault().getTable("Color")
        table.putValue("self.color", self.color)

        # use the network table to log our data
        fms_table = ntcore.NetworkTableInstance.getDefault().getTable("FMSInfo")
        is_red = fms_table.getBoolean("IsRedAlliance", True)
        if is_red:
            color_now = DriverStation.Alliance.kRed
        else:
            color_now = DriverStation.Alliance.kBlue
    
        # if wrong color -> flip -> update field table
        if not color_now == self.color: # if wrong color
            Field.flip_poses() # flip. 
            self.color = color_now
            Field.update_field_table() # Update field table for usage in diagnostics
        
        if self.isSimulation():
            wpilib.DriverStation.silenceJoystickConnectionWarning(True)

        # run the scheduler if we are not in debug mode
        if not config.DEBUG_MODE:
            try:
                # pass
                self.scheduler.run()
            except Exception as e:
                self.log.error(e)
                self.nt.getTable("errors").putString("command scheduler", str(e))
        else:
            try:
                # pass
                self.scheduler.run()
            except Exception as e:
                self.log.error(e)
                self.nt.getTable("errors").putString("command scheduler", str(e))
                raise e


    # called once in the transition from any state into teleop
    def teleopInit(self):
        Keymap.Drivetrain.cool_button_motorspin.onTrue(
            self.robot.drivetrain.move_motor()
        ).onFalse(self.robot.drivetrain.stop_motor())
        
        self.log.info("Teleop initialized")

    # set the specific period timing for teleop
    def teleopPeriodic(self):
        pass

    # called once as we transition from any state into auto
    def autonomousInit(self):
        pass
    # set the specific period timing for auto
    def autonomousPeriodic(self):
        pass

    # at the end of auto set the x and y velocity to 0 and the angle to forward and cancel the auto path
    def autonomousExit(self):
        # pass
        self.scheduler.cancelAll()

    # called once when the robot is entering or in the state of "Disabled"
    def disabledInit(self) -> None:
        self.log.info("Robot disabled")

    # set the sepcific period timing for disabled
    def disabledPeriodic(self) -> None:
        pass

# causes the code to run
if __name__ == "__main__":
    wpilib.run(_Robot)
