package frc.robot.subsystems;

import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkLowLevel;
import com.revrobotics.spark.SparkMax;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

// picks balls off the ground
public class IndexerSubsystem extends SubsystemBase {

    private final SparkFlex indexerMotor;
    private double speed;

    public IndexerSubsystem() {
        if (Constants.Intake.disabled) {
            indexerMotor = null;
            return;
        }

        // set up motors
        indexerMotor = new SparkFlex(Constants.Indexer.indexerMotorChannel, SparkLowLevel.MotorType.kBrushless);
        speed = 0.0;
    }

    public void setSpeed(double speed) {
        this.speed = speed;
    }

    @Override
    public void periodic() {
        if (Constants.Intake.disabled) return;

        // set motor speeds
        indexerMotor.set(speed);

    }
}
