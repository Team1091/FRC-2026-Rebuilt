package frc.util;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.units.measure.Angle;

import static edu.wpi.first.units.Units.Radians;

public final class GeometryUtil {
    public static boolean isNear(Rotation2d expected, Rotation2d actual, Angle tolerance) {
        final double expectedRadians = MathUtil.angleModulus(expected.getRadians());
        final double actualRadians = MathUtil.angleModulus(actual.getRadians());
        return MathUtil.isNear(expectedRadians, actualRadians, tolerance.in(Radians), -Math.PI, Math.PI);
    }
}
