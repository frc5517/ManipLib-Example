package maniplib.utils;

import edu.wpi.first.math.controller.ElevatorFeedforward;
import edu.wpi.first.math.system.plant.DCMotor;

public class ManipElevatorConfig {

  public DCMotor manipMotorType;

  public ElevatorFeedforward elevatorFeedforward;

  public double elevatorGearing;

  public double elevatorDrumRadius;

  public double elevatorCarriageMass;

  public double maxHeight;

  public double minHeight;

  public double elevatorWidth;

  public double elevatorHeight;

  public ManipElevatorConfig(
      DCMotor manipMotorType,
      ElevatorFeedforward elevatorFeedforward,
      double elevatorGearing,
      double elevatorDrumRadius,
      double elevatorCarriageMass,
      double maxHeight,
      double minHeight,
      double elevatorWidth,
      double elevatorHeight) {
    this.manipMotorType = manipMotorType;
    this.elevatorFeedforward = elevatorFeedforward;
    this.elevatorGearing = elevatorGearing;
    this.elevatorDrumRadius = elevatorDrumRadius;
    this.elevatorCarriageMass = elevatorCarriageMass;
    this.maxHeight = maxHeight;
    this.minHeight = minHeight;
    this.elevatorWidth = elevatorWidth;
    this.elevatorHeight = elevatorHeight;
  }
}
