package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import maniplib.ManipShooterIntake;
import maniplib.motors.ManipSparkMax;
import maniplib.utils.PIDControlType;

public class IntakeShooterSubsystem extends SubsystemBase {

    private final ManipSparkMax shooterIntakeMotor = new ManipSparkMax(0);
    private final ManipShooterIntake shooterIntake = new ManipShooterIntake(shooterIntakeMotor);

    /**
     * Subsystem Constructor.
     */
    public IntakeShooterSubsystem() {
        shooterIntakeMotor.setMotorBrake(true);
    }

    /**
     * Use velocity PID Control to run the motor.
     */
    public Command setVelocity() {
        return run(() -> {
            shooterIntake.setReference(Constants.IntakeShooterConstants.velocitySetpoint);
        });
    }

    /**
     * {@link ManipShooterIntake} {@link PIDControlType} is velocity by default.
     * This is just an example of how to change it.
     */
    public Command setPosition(double position) {
        return runEnd(() -> {
            shooterIntakeMotor.setPIDControlType(PIDControlType.ControlType.POSITION);
            shooterIntake.setReference(position);
        }, () -> {
            shooterIntakeMotor.setPIDControlType(PIDControlType.ControlType.VELOCITY);
        });
    }

    /**
     * Intake with given speed.
     */
    public Command intake() {
        return run(() -> shooterIntake.setSpeed(-Constants.IntakeShooterConstants.intakeSpeed));
    }

    /**
     * Shoot with given speed.
     */
    public Command shoot() {
        return run(() -> shooterIntake.setSpeed(Constants.IntakeShooterConstants.intakeSpeed));
    }

}
