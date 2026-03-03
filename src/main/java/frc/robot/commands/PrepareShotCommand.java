package frc.robot.commands;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.interpolation.InterpolatingDoubleTreeMap;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.subsystems.HoodSubsystem;

import java.util.function.Supplier;

public class PrepareShotCommand extends Command {

    final static InterpolatingDoubleTreeMap shooterMap = new InterpolatingDoubleTreeMap();

    static {
        // we have a set of data on how far we can shoot
        shooterMap.put(10.0, 0.4); // 10 meters,  3000 RPM
        shooterMap.put(20.0, 0.6); // 20 meters, 4000 RPM
    }

    private final Supplier<Pose2d> pose2dSupplier;
    final HoodSubsystem hoodSubsystem;

    public PrepareShotCommand(
            HoodSubsystem hoodSubsystem,
            Supplier<Pose2d> pose2dSupplier
    ) {
        this.pose2dSupplier = pose2dSupplier;
        this.hoodSubsystem = hoodSubsystem;
        addRequirements(hoodSubsystem);
    }


    @Override
    public void execute() {
        var ourPose = pose2dSupplier.get();
        var distanceToTargetMeters = ourPose.getTranslation().getDistance(Constants.Locations.hubPose.get().getTranslation());

        double hoodAngle = shooterMap.get(distanceToTargetMeters);
        hoodSubsystem.setTargetAngle(hoodAngle);
    }


    @Override
    public boolean isFinished() {
        return hoodSubsystem.isCloseEnoughToTarget();
    }

}
