package frc.robot.enums;

public enum IntakeState {
    RETRACTED(0.0, 0.0),
    EXTENDED(0.0, 0.5),
    HARVEST(1.0, 0.5);

    IntakeState(Double intakeMotorSpeed, Double extenderMotorPosition) {
        this.intakeMotorSpeed = intakeMotorSpeed;
        this.extenderMotorPosition = extenderMotorPosition;
    }

    public final double intakeMotorSpeed;
    public final double extenderMotorPosition;
}