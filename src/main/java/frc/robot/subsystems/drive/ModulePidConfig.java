package frc.robot.subsystems.drive;

public record ModulePidConfig(
        FeedForwardParams driveFeedForward,
        PidConfig drivePid,
        PidConfig turnPid


) {


}
