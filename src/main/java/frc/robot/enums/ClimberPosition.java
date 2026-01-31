package frc.robot.enums;

public enum ClimberPosition {
    UP(1.0),
    DOWN(0.0);

    ClimberPosition(double pos) {
        climberPosition = pos;
    }

    public final double climberPosition;
}
