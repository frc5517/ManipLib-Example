package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import maniplib.ManipElevator;
import maniplib.motors.ManipSparkMax;

public class ElevatorSubsystem extends SubsystemBase {
    private final ManipSparkMax leftElevatorMotor = new ManipSparkMax(2);
    private final ManipSparkMax rightElevatorMotor = new ManipSparkMax(3);
    private final ManipElevator elevator = new ManipElevator(leftElevatorMotor);

    /**
     * Subsystem Constructor.
     */
    public ElevatorSubsystem() {
        elevator.addFollower(rightElevatorMotor, true);
    }

    /**
     * Use PID Control to run the motor.
     */
    public Command toSetpoint() {
        return run(() -> elevator.setReference(Constants.ElevatorConstants.elevatorSetpoint));
    }

    /**
     * Move up with given speed.
     */
    public Command moveUp() {
        return run(() -> elevator.setSpeed(Constants.ElevatorConstants.elevatorSpeed));
    }

    /**
     * Move down with given speed.
     */
    public Command moveDown() {
        return run(() -> elevator.setSpeed(-Constants.ElevatorConstants.elevatorSpeed));
    }

}
