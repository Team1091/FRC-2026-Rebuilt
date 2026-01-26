package frc.robot;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;

import java.util.Optional;

import static edu.wpi.first.units.Units.Inches;

public class Landmarks {
    public static Translation2d hubPosition() {
        final Optional<Alliance> alliance = DriverStation.getAlliance();
        if (alliance.isPresent() && alliance.get() == Alliance.Blue) {
            return new Translation2d(Inches.of(182.105), Inches.of(158.845));
        }
        return new Translation2d(Inches.of(469.115), Inches.of(158.845));
    }
}
