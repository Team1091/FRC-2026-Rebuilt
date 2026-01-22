package frc.util;

import edu.wpi.first.units.measure.Time;
import edu.wpi.first.wpilibj.Timer;

import static edu.wpi.first.units.Units.Seconds;

public class Stopwatch {
    private double startTimeInSeconds = Double.POSITIVE_INFINITY;

    public void start() {
        startTimeInSeconds = Timer.getFPGATimestamp();
    }

    public void startIfNotRunning() {
        if (Double.isInfinite(startTimeInSeconds)) {
            start();
        }
    }

    public void reset() {
        startTimeInSeconds = Double.POSITIVE_INFINITY;
    }

    public double elapsedSeconds() {
        if (Double.isInfinite(startTimeInSeconds)) {
            return 0.0;
        }
        return Timer.getFPGATimestamp() - startTimeInSeconds;
    }

    public Time elapsedTime() {
        return Seconds.of(elapsedSeconds());
    }
}
