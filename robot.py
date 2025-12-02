# import standard Python libraries
import math

# import key robotics libraries
import commands2
import ntcore
import phoenix6 as ctre
import wpilib
#REDUNDANT? import wpilib.drive


# import our libraries
import command
import config
import autos
#REDUNDANT? import autos.auto_routine

# import the variable Subsytem from the toolkit library
from toolkit.subsystem import Subsystem

# import constants
from robot_systems import (  # noqa
    Robot,
    Pneumatics,
    Sensors,
    LEDs,
    PowerDistribution,
    Field,
)

# import the variables DriverStation and SenableChooser from wpilib for ease of use
from wpilib import DriverStation, SendableChooser

# import the variables
import utils
from oi.OI import OI
from pathplannerlib.auto import PathPlannerPath, FollowPathCommand, AutoBuilder
from wpimath.geometry import Pose2d, Rotation2d, Transform2d
from utils import get_red_pose

# defining the class "_Robot"
class _Robot(wpilib.TimedRobot):
    def __init__(self):
        super().__init__()

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

        # flips the field poses from blue to red alliance field poses
        Field.flip_poses()
        Field.update_field_table("Field")
        self.log.info(f"Scheduler period set to {period} seconds")

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
        except Exception as e:
            self.log.error(e)
            self.nt.getTable("errors").putString("subsystem init", str(e))
            raise e
        
        # define our autos
        self.auto_selection = SendableChooser()
        self.auto_selection.setDefaultOption("kenny path", autos.kenny)

        self.auto_selection.addOption("Three L4 Right", autos.three_l4_right)
        self.auto_selection.addOption("Three L4 Left", autos.three_l4_left)
        self.auto_selection.addOption("Bump", autos.three_l4_left_bump)
        self.auto_selection.addOption("Leave", autos.leave)
        self.auto_selection.addOption("Center", autos.dealgae_center)

        # allow us to choose our auto in Smart Dashboard
        wpilib.SmartDashboard.putData("Auto", self.auto_selection)

        # harmonize our bus utilization frequencies
        ctre.hardware.ParentDevice.optimize_bus_utilization_for_all()
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

        # current_alliance = DriverStation.getAlliance()
        if not color_now == self.color:
            Field.flip_poses()
            self.color = color_now
            Field.update_field_table()
        if self.isSimulation():
            wpilib.DriverStation.silenceJoystickConnectionWarning(True)

        # run the scheduler if we are not in debug mode
        if not config.DEBUG_MODE:
            try:
                self.scheduler.run()
            except Exception as e:
                self.log.error(e)
                self.nt.getTable("errors").putString("command scheduler", str(e))
        else:
            try:
                self.scheduler.run()
            except Exception as e:
                self.log.error(e)
                self.nt.getTable("errors").putString("command scheduler", str(e))
                raise e

        # Field.odometry.disable()

        # update the field odometry
        pose = Field.odometry.update()

        # log the pose
        self.nt.getTable("Odometry").putNumberArray(
            "Estimated pose", [pose.X(), pose.Y(), pose.rotation().radians()]
        )
        
        # log the swerve module states including estimated pose of the robot with rotation, etc.
        Robot.drivetrain.update_tables()
        Sensors.cam_controller.update_tables()
        ...

    # Initialize subsystems
    # Pneumatics

    # called once in the transition from any state into teleop
    def teleopInit(self):
        OI.init()
        OI.map_controls()
        self.scheduler.schedule(commands2.SequentialCommandGroup(
                command.DrivetrainZero(Robot.drivetrain),
                command.DriveSwerveCustom(Robot.drivetrain)
        ))
        # self.scheduler.schedule(commands2.SequentialCommandGroup(
        #     command.SetWrist(Robot.wrist, 0),
        #     # command.SetElevator(Robot.elevator, 0),
        # ))
        # self.scheduler.schedule(
        #     command.DeployClimb(Robot.climber, upper_bound=config.climb_initial_out).onlyIf(lambda: Robot.climber.get_motor_revolutions() <= 30)
        # )
        self.log.info("Teleop initialized")

    # set the specific period timing for teleop
    def teleopPeriodic(self):
        pass

    # called once as we transition from any state into auto
    def autonomousInit(self):
        auto: autos.AutoRoutine = self.auto_selection.getSelected()
        starting_pose: Pose2d = auto.blue_start_pose if DriverStation.getAlliance() == DriverStation.Alliance.kBlue else auto.red_start_pose
        Robot.drivetrain.reset_odometry_auto(starting_pose)
        self.scheduler.schedule(commands2.SequentialCommandGroup(
            command.DrivetrainZero(Robot.drivetrain, starting_pose.rotation().radians()),
            commands2.ParallelCommandGroup(
                auto.command,
                command.DeployClimb(Robot.climber, upper_bound=config.climb_initial_out)
            )
            
        ))
        # log that we have successfully entered auto
        self.log.info("Autonomous initialized")

    # set the specific period timing for auto
    def autonomousPeriodic(self):
        pass

    # at the end of auto set the x and y velocity to 0 and the angle to forward and cancel the auto path
    def autonomousExit(self):
        Robot.drivetrain.set_driver_centric((0, 0), 0)
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
