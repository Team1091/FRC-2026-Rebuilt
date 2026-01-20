package frc.robot.subsystems.drive;

import edu.wpi.first.math.controller.SimpleMotorFeedforward;

public record FeedForwardParams(
        double kS, double kV
) {
    public SimpleMotorFeedforward toSimpleMotorFeedforward() {
        return new SimpleMotorFeedforward(kS, kV);
    }
}
