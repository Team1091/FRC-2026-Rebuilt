package frc.robot.enums;

import edu.wpi.first.math.geometry.Pose2d;

import static frc.robot.commands.Autos.centerStartingPose;
import static frc.robot.commands.Autos.leftStartingPose;
import static frc.robot.commands.Autos.rightStartingPose;

public enum StartPosish {
    LEFT(leftStartingPose),
    RIGHT(rightStartingPose),
    CENTER(centerStartingPose);

    StartPosish(Pose2d startingPose) {
        this.startingPose = startingPose;
    }

    public Pose2d startingPose;
}
