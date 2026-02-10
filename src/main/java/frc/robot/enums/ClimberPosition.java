package frc.robot.enums;

public enum ClimberPosition {
    UP(180.0),
    DOWN(0.0);

    ClimberPosition(double pos) {
        position = pos;
    }

    public final double position;
}
