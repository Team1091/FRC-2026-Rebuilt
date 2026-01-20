// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Constants.OperatorConstants;
import frc.robot.commands.Autos;
import frc.robot.commands.DriveCommand;
import frc.robot.subsystems.PoseEstimationSubsystem;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.drive.GyroIOPigeon2;
import frc.robot.subsystems.drive.ModuleIOTalonFX;

import static frc.robot.Constants.Swerve.BACK_LEFT;
import static frc.robot.Constants.Swerve.BACK_RIGHT;
import static frc.robot.Constants.Swerve.FRONT_LEFT;
import static frc.robot.Constants.Swerve.FRONT_RIGHT;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and trigger mappings) should be declared here.
 */
public class RobotContainer {
    // The robot's subsystems and commands are defined here...
//  private final ExampleSubsystem m_exampleSubsystem = new ExampleSubsystem();
    private final Drive drive;
    private final PoseEstimationSubsystem poseEstimationSubsystem;


    // Replace with CommandPS4Controller or CommandJoystick if needed
    private final CommandXboxController driverController =
            new CommandXboxController(OperatorConstants.kDriverControllerPort);

    // This lets us select the command to run in autonomous
    private SendableChooser<Command> autoChooser;

    /**
     * The container for the robot. Contains subsystems, OI devices, and commands.
     */
    public RobotContainer() {

        drive = new Drive(
                new GyroIOPigeon2(),//change if using different gyro
                new ModuleIOTalonFX(FRONT_LEFT),
                new ModuleIOTalonFX(FRONT_RIGHT),
                new ModuleIOTalonFX(BACK_LEFT),
                new ModuleIOTalonFX(BACK_RIGHT)
        );
        poseEstimationSubsystem = new PoseEstimationSubsystem(
                drive::getGyroRotation,
                drive::getModulePositions
        );

        // TODO: set up more subsystems
        // IntakeSubsystem - picks balls off the ground
        // IndexSubsystem
        // LaunchSubSystem - spins up a flywheel to launch the balls
        // ClimberSubsystem - climbs up, holds, and back down


        configureAutonomous();
        // Configure the trigger bindings
        configureBindings();
    }

    // Set the defaults when powered on
    public void robotInit() {
        // TODO: this sets the pose estimate to (0,0)
        //  which seems like it would think its in the corner till it detects a target.
        //  If we can start this off thinking its in a valid starting position (dropdown for starting position)
        // It would be a lot more accurate for autonomous
        poseEstimationSubsystem.setCurrentPose(new Pose2d(new Translation2d(0, 0), new Rotation2d(0)));
        drive.resetGyro();
        drive.setIsFieldOriented(true);

        //TODO: set up camera capture
//    CameraServer.startAutomaticCapture().setExposureManual(40);
//    Shuffleboard.getTab("General").add("Camera", 0).withWidget(BuiltInWidgets.kCameraStream);

//    FollowPathCommand.warmupCommand().schedule();
    }

    public void robotEnabled() {
//    climberSubsystem.resetEncoder();
        drive.straightenWheels();
    }

    private void configureAutonomous() {
        autoChooser = new SendableChooser<>();
        autoChooser.addOption("Spin Wildly", Autos.spinAuto(drive));
    }

    /**
     * Use this method to define your trigger->command mappings. Triggers can be created via the
     * {@link Trigger#Trigger(java.util.function.BooleanSupplier)} constructor with an arbitrary
     * predicate, or via the named factories in {@link
     * edu.wpi.first.wpilibj2.command.button.CommandGenericHID}'s subclasses for {@link
     * CommandXboxController Xbox}/{@link edu.wpi.first.wpilibj2.command.button.CommandPS4Controller
     * PS4} controllers or {@link edu.wpi.first.wpilibj2.command.button.CommandJoystick Flight
     * joysticks}.
     */
    private void configureBindings() {

        // x+ forward is front, x- is backward
        drive.setDefaultCommand(
                DriveCommand.joystickDrive(
                        drive,
                        driverController::getLeftY,
                        () -> { // y+ is to the left, y- is to the right
                            return -driverController.getLeftX();
                        },
                        () -> { // z+ is rotating counterclockwise
                            return -driverController.getRightX();
                        }
                )
        );

        // TODO: add bindings
        //  driver.rightBumper().whileTrue(new WheelCommand(chuteSubsystem, Constants.Chute.shootSpeed));


    }

    /**
     * Use this to pass the autonomous command to the main {@link Robot} class.
     *
     * @return the command to run in autonomous
     */
    public Command getAutonomousCommand() {
        // An example command will be run in autonomous
        return autoChooser.getSelected();
    }
}
