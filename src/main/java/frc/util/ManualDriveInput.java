package frc.util;

public record ManualDriveInput(double forward, double left, double rotation) {

    public ManualDriveInput() {
        this(0, 0, 0);
    }

    public boolean hasTranslation() {
        return Math.hypot(forward, left) > 0;
    }

    public boolean hasRotation() {
        return Math.abs(rotation) > 0;
    }
}
