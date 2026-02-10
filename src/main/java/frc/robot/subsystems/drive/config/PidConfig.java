package frc.robot.subsystems.drive.config;

import edu.wpi.first.math.controller.PIDController;

public record PidConfig(
        double kP, double kI, double kD
) {
    public PIDController toPidController() {
        return new PIDController(kP, kI, kD);
    }
}
