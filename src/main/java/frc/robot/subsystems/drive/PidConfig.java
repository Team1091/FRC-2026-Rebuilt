package frc.robot.subsystems.drive;

import edu.wpi.first.math.controller.PIDController;

public record PidConfig(
        double kP, double kI, double kD
) {
    PIDController toPidController() {
        return new PIDController(kP, kI, kD);
    }
}
