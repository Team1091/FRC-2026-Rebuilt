package frc.robot.subsystems;

import com.revrobotics.spark.SparkLowLevel;
import com.revrobotics.spark.SparkMax;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.enums.ShooterState;

// spins up a flywheel to launch the balls, indexes them
public class ShooterSubsystem extends SubsystemBase {

    private final SparkMax leftShooterMotor;
    private final SparkMax rightShooterMotor;
    private final SparkMax indexerMotor;

    private ShooterState shooterState = ShooterState.IDLE;

    public ShooterSubsystem() {
        if (Constants.Shooter.disabled) {
            leftShooterMotor = null;
            rightShooterMotor = null;
            indexerMotor = null;
            return;
        }
        leftShooterMotor = new SparkMax(Constants.Shooter.leftMotorChannel, SparkLowLevel.MotorType.kBrushless);
        rightShooterMotor = new SparkMax(Constants.Shooter.rightMotorChannel, SparkLowLevel.MotorType.kBrushless);
        indexerMotor = new SparkMax(Constants.Shooter.indexerMotorChannel, SparkLowLevel.MotorType.kBrushless);
    }


    public void setShooterState(ShooterState shooterState) {
        this.shooterState = shooterState;
    }

    // TODO: we will need to add controls for the aiming hood, and probably calculate a shooting solution

    @Override
    public void periodic() {
        if (Constants.Shooter.disabled) return;

        leftShooterMotor.set(shooterState.flywheelSpeed);
        rightShooterMotor.set(shooterState.flywheelSpeed);
        indexerMotor.set(shooterState.indexSpeed);
    }

}
