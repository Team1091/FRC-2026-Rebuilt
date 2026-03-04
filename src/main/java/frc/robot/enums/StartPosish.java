package frc.robot.enums;

import frc.robot.utils.FlipPose2d;

import static frc.robot.commands.Autos.centerStartingPose;
import static frc.robot.commands.Autos.leftStartingPose;
import static frc.robot.commands.Autos.rightStartingPose;

public enum StartPosish {
    LEFT(leftStartingPose),
    RIGHT(rightStartingPose),
    CENTER(centerStartingPose);

    StartPosish(FlipPose2d startingPose) {
        this.startingPose = startingPose;
    }

    public final FlipPose2d startingPose;
}
