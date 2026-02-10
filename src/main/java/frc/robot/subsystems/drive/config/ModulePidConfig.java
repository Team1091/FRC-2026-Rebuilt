package frc.robot.subsystems.drive.config;

public record ModulePidConfig(
        FeedForwardParams driveFeedForward,
        PidConfig drivePid,
        PidConfig turnPid
) {
}