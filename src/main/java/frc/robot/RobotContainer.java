// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Constants.OperatorConstants;
import frc.robot.commands.Autos;
import frc.robot.commands.DriveCommand;
import frc.robot.commands.DriveWhilePointingAtCommand;
import frc.robot.commands.IntakeCommand;
import frc.robot.commands.ManualClimbCommand;
import frc.robot.commands.PivotCommand;
import frc.robot.commands.PrepareShotCommand;
import frc.robot.commands.ShooterCommand;
import frc.robot.enums.ShooterState;
import frc.robot.enums.StartPosish;
import frc.robot.subsystems.ClimberSubsystem;
import frc.robot.subsystems.HoodSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.PivotSubsystem;
import frc.robot.subsystems.PoseEstimationSubsystem;
import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.drive.gyro.GyroIOPigeon2;
import frc.robot.subsystems.drive.module.ModuleIOTalonFX;

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
    private final Drive drive;
    private final PoseEstimationSubsystem poseEstimationSubsystem;
    private final ClimberSubsystem climberSubsystem;
    private final IntakeSubsystem intakeSubsystem;
    private final ShooterSubsystem shooterSubsystem;
    private final HoodSubsystem hoodSubsystem;
    private final PivotSubsystem pivotSubsystem;

    // Replace with CommandPS4Controller or CommandJoystick if needed
    private final CommandXboxController driverController = new CommandXboxController(OperatorConstants.kDriverControllerPort);

    // This lets us select the command to run in autonomous
    private SendableChooser<Command> autoChooser;
    private SendableChooser<StartPosish> startPosChooser;

    /**
     * The container for the robot. Contains subsystems, OI devices, and commands.
     */
    public RobotContainer() {
        drive = new Drive(
                new GyroIOPigeon2(),//change if using different gyro
                new ModuleIOTalonFX(Constants.Swerve.moduleHardware[FRONT_LEFT]),
                new ModuleIOTalonFX(Constants.Swerve.moduleHardware[FRONT_RIGHT]),
                new ModuleIOTalonFX(Constants.Swerve.moduleHardware[BACK_LEFT]),
                new ModuleIOTalonFX(Constants.Swerve.moduleHardware[BACK_RIGHT])
        );
        poseEstimationSubsystem = new PoseEstimationSubsystem(
                drive::getGyroRotation,
                drive::getModulePositions
        );
        drive.setPoseEstimationSubSystem(poseEstimationSubsystem);

        intakeSubsystem = new IntakeSubsystem();
        shooterSubsystem = new ShooterSubsystem();
        hoodSubsystem = new HoodSubsystem();
        climberSubsystem = new ClimberSubsystem();
        pivotSubsystem = new PivotSubsystem();

        configureStartingPositions();
        configureAutonomous();
        // Configure the trigger bindings
        configureBindings();
    }

    // Set the defaults when powered on
    public void robotInit() {

        // Initialize this to the center, we reset if it changes
        poseEstimationSubsystem.setCurrentPose(StartPosish.CENTER.startingPose);
        drive.resetGyro();
        drive.setIsFieldOriented(true);

        if (!Constants.Camera.disabled) {
            CameraServer.startAutomaticCapture().setExposureManual(40);
            Shuffleboard.getTab("General").add("Camera", 0).withWidget(BuiltInWidgets.kCameraStream);
        }
    }

    public void robotEnabled() {
        hoodSubsystem.resetEncoder();
        climberSubsystem.resetEncoders();
        drive.straightenWheels();
        pivotSubsystem.resetEncoder();
    }

    private void configureStartingPositions() {
        startPosChooser = new SendableChooser<>();
        startPosChooser.addOption("Left", StartPosish.LEFT);
        startPosChooser.setDefaultOption("Center", StartPosish.CENTER);
        startPosChooser.addOption("Right", StartPosish.RIGHT);
        SmartDashboard.putData("Start Position", startPosChooser);

        poseEstimationSubsystem.setCurrentPose(StartPosish.CENTER.startingPose);
        startPosChooser.onChange((startPosish) -> {
            poseEstimationSubsystem.setCurrentPose(startPosish.startingPose);
        });
    }

    private void configureAutonomous() {
        autoChooser = new SendableChooser<>();
        autoChooser.setDefaultOption("Do Nothing", new WaitCommand(1.0));
        autoChooser.addOption("Spin Wildly", Autos.spinAuto(drive));
        autoChooser.addOption("Drive Forward", Autos.driveForward(drive));
        autoChooser.addOption("Manual Align", Autos.manualAlign(drive));
        autoChooser.addOption("Win", Autos.swoopThroughMiddleThenShoot(
                drive,
                hoodSubsystem,
                shooterSubsystem,
                pivotSubsystem,
                climberSubsystem,
                poseEstimationSubsystem,
                StartPosish.RIGHT
        ));
        autoChooser.addOption("Drive To Opponent", Autos.driveToOpponent(
                drive,
                hoodSubsystem,
                shooterSubsystem,
                poseEstimationSubsystem,
                StartPosish.RIGHT
        ));
        autoChooser.addOption("Human Pickup", Autos.humanPickup(
                drive,
                hoodSubsystem,
                shooterSubsystem,
                climberSubsystem,
                poseEstimationSubsystem
        ));
        SmartDashboard.putData("Auto Mode", autoChooser);
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
//                        () -> 0.5,
                        () -> { // y+ is to the left, y- is to the right
                            return -driverController.getLeftX();
                        },
                        () -> { // z+ is rotating counterclockwise
                            return -driverController.getRightX();
                        }
                )
        );

        // Aim while driving
        driverController.leftTrigger().whileTrue(
                new ParallelCommandGroup(
                        new DriveWhilePointingAtCommand(
                                drive,
                                poseEstimationSubsystem,
                                Constants.Locations.hubPose,
                                () -> { // y+ is to the left, y- is to the right
                                    return -driverController.getLeftX();
                                },
                                () -> { // z+ is rotating counterclockwise
                                    return -driverController.getRightX();
                                }
                        ),
                        new PrepareShotCommand(hoodSubsystem, poseEstimationSubsystem::getCurrentPose)
                )
        );
        // spin up while shooting
        driverController.leftTrigger().whileTrue(new ShooterCommand(shooterSubsystem, ShooterState.SPIN_UP));
        driverController.y().whileTrue(new ShooterCommand(shooterSubsystem, ShooterState.SPIN_UP));
        driverController.rightTrigger().whileTrue(new ShooterCommand(shooterSubsystem, ShooterState.FIRE));

        // Climber uses bumpers
        driverController.leftBumper().whileTrue((new ManualClimbCommand(climberSubsystem, Constants.Climber.climbingSpeed)));
        driverController.rightBumper().whileTrue((new ManualClimbCommand(climberSubsystem, -Constants.Climber.climbingSpeed)));

        // TODO: add additional bindings

        // Intake
//        driverController.a().whileTrue(new RunIntakeCommand(intakeSubsystem, IntakeState.HARVEST));
        driverController.x().whileTrue(new IntakeCommand(intakeSubsystem, Constants.Intake.intakeSpeed));
        driverController.a().whileTrue(new PivotCommand(pivotSubsystem, Constants.Pivot.pivotSpeedOut));
        driverController.b().whileTrue(new PivotCommand(pivotSubsystem, Constants.Pivot.pivotSpeedIn));


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
