package frc.robot;

import com.revrobotics.spark.SparkBase;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.wpilibj2.command.Command;
import maniplib.ManipArm;
import maniplib.ManipShooterIntake;
import maniplib.motors.ManipSparkMax;

import static edu.wpi.first.wpilibj2.command.Commands.run;

public class SuperStructure {

    private final ManipSparkMax shooterIntakeMotor = new ManipSparkMax(0, DCMotor.getNeo550(1));
    private final ManipShooterIntake shooterIntake = new ManipShooterIntake(shooterIntakeMotor);

    // Null motorType defaults to DCMotor.getNEO(numMotors: 1)
    private final ManipSparkMax armMotor = new ManipSparkMax(1, null);
    private final ManipArm arm = new ManipArm(armMotor);

    private final ManipSparkMax elevatorMotor = new ManipSparkMax(2, DCMotor.getNEO(2));
    private final ManipSparkMax elevatorMotor2 = new ManipSparkMax(3, null);
    private final ManipArm elevator = new ManipArm(elevatorMotor);

    public SuperStructure() {
        elevator.addFollower(elevatorMotor2, true);

        shooterIntakeMotor.setMotorBrake(true);
        shooterIntakeMotor.setInverted(true);
    }

    public Command moveElevatorAndArmThenScore() {
        return run(() -> {
            elevator.setReference(5).
                    alongWith(arm.setReference(15))
                    .andThen(shooterIntake.setReference(100, SparkBase.ControlType.kVelocity))
                    .withTimeout(2);
        });
    }

}
