package frc.robot.enums;

public enum IntakeState {
    RETRACTED(0.0), // starting
    EXTENDED(0.0),
    HARVEST(1.0);

    IntakeState(Double intakeMotorSpeed) {
        this.intakeMotorSpeed = intakeMotorSpeed;
    }
    public final double intakeMotorSpeed;
}