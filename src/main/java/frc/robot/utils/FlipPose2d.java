package frc.robot.utils;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import frc.robot.Constants;

import static frc.robot.RobotContainer.isOnRed;

public class FlipPose2d {
    private final Pose2d blue;
//    private final Pose2d red;

    public FlipPose2d(Pose2d blue) {
        this.blue = blue;

    }

    public FlipPose2d(Translation2d translation, Rotation2d rotation) {
        this.blue = new Pose2d(translation, rotation);
//        red = blue.rotateAround(Constants.Locations.center, Rotation2d.kCW_Pi_2);
    }

    public FlipPose2d(double x, double y, Rotation2d rotation) {
        this.blue = new Pose2d(x, y, rotation);
//        red = blue.rotateAround(Constants.Locations.center, Rotation2d.kCW_Pi_2);
    }


    public Pose2d get() {
        if (isOnRed()) {
            return blue.rotateAround(Constants.Locations.center, Rotation2d.kCW_Pi_2);
        } else
            return blue;
    }
}
