package frc.robot.subsystems;

import com.revrobotics.PersistMode;
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkLowLevel;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkFlexConfig;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.enums.ShooterState;

// spins up a flywheel to launch the balls, indexes them
public class ShooterSubsystem extends SubsystemBase {

//    private final SparkFlex leftShooterMotor;
//    private final SparkFlex rightShooterMotor;
    private final SparkMax indexerMotor;
    private final SparkMax loaderMotor;

    private ShooterState shooterState = ShooterState.IDLE;

    public ShooterSubsystem() {
        if (Constants.Shooter.disabled) {
//            leftShooterMotor = null;
//            rightShooterMotor = null;
            indexerMotor = null;
            loaderMotor = null;
            return;
        }

//        leftShooterMotor = new SparkFlex(Constants.Shooter.leftMotorChannel, SparkLowLevel.MotorType.kBrushless);
//        rightShooterMotor = new SparkFlex(Constants.Shooter.rightMotorChannel, SparkLowLevel.MotorType.kBrushless);

        // Reverse the right motor
        var rightConfig = new SparkFlexConfig();
        rightConfig.inverted(true);
//        rightShooterMotor.configure(rightConfig, com.revrobotics.ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

        indexerMotor = new SparkMax(Constants.Shooter.indexerMotorChannel, SparkLowLevel.MotorType.kBrushless);
        loaderMotor = new SparkMax(Constants.Shooter.loaderMotorChannel, SparkLowLevel.MotorType.kBrushless);
    }


    public void setShooterState(ShooterState shooterState) {
        this.shooterState = shooterState;
    }

    @Override
    public void periodic() {
        if (Constants.Shooter.disabled) return;

//        leftShooterMotor.set(shooterState.flywheelSpeed);
//        rightShooterMotor.set(shooterState.flywheelSpeed);
        indexerMotor.set(shooterState.indexSpeed);
        loaderMotor.set(shooterState.indexSpeed);
    }

}
