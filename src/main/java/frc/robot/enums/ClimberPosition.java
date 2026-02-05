package frc.robot.enums;

public enum ClimberPosition {
    UP(180.0),
    DOWN(0.0);

    ClimberPosition(double pos) {
        climberPosition = pos;
    }

    public final double climberPosition;
}
