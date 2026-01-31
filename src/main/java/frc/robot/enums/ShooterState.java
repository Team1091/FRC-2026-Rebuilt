package frc.robot.enums;

public enum ShooterState {

    IDLE(0.0, 0.0),
    SPIN_UP(1.0, 0.0),
    FIRE(1.0, 1.0);

    ShooterState(double flywheelSpeed, double indexSpeed) {
        this.flywheelSpeed = flywheelSpeed;
        this.indexSpeed = indexSpeed;
    }

    public final double flywheelSpeed;
    public final double indexSpeed;

}
