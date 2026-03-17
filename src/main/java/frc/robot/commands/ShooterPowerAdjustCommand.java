package frc.robot.commands;

import edu.wpi.first.math.interpolation.InterpolatingDoubleTreeMap;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.ManualShooterSubsystem;
import frc.robot.subsystems.PoseEstimationSubsystem;

import static edu.wpi.first.math.MathUtil.clamp;
import static frc.robot.Constants.Locations.hubPose;

public class ShooterPowerAdjustCommand extends Command {


    final static InterpolatingDoubleTreeMap shooterMap = new InterpolatingDoubleTreeMap();

    static {
        // we have a set of data on how far we can shoot
        shooterMap.put(Units.feetToMeters(4.0), 0.4); // meters, to shooter power
        shooterMap.put(Units.feetToMeters(5.0), 0.5);
        shooterMap.put(Units.feetToMeters(8.0), 0.8);
    }

    private final ManualShooterSubsystem manualShooterSubsystem;
    private final PoseEstimationSubsystem poseEstimationSubsystem;

    public ShooterPowerAdjustCommand(
            ManualShooterSubsystem manualShooterSubsystem,
            PoseEstimationSubsystem poseEstimationSubsystem
    ) {
        this.manualShooterSubsystem = manualShooterSubsystem;
        this.poseEstimationSubsystem = poseEstimationSubsystem;
        addRequirements(manualShooterSubsystem);
    }

    @Override
    public void execute() {
        var distanceInMeters = poseEstimationSubsystem.getCurrentPose().getTranslation().getDistance(hubPose.get().getTranslation());

        var targetSpeed = shooterMap.get(distanceInMeters);
        manualShooterSubsystem.setSpeed(clamp(targetSpeed, 0, 1));
    }
}
