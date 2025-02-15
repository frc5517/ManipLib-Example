package frc.robot;

import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.wpilibj.smartdashboard.Mechanism2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismLigament2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismRoot2d;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj.util.Color8Bit;
import maniplib.utils.ManipArmConstants;
import maniplib.utils.ManipElevatorConstants;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Meters;

public class Constants {

    public static final Mechanism2d         sideRobotView = new Mechanism2d(ArmConstants.armConfig.kArmLength * 2,
            ElevatorConstants.elevatorConfig.kMaxHeight.in(
                    Meters) +
                    ArmConstants.armConfig.kArmLength);

    public static final MechanismRoot2d kElevatorCarriage;
    public static final MechanismLigament2d kArmMech;
    public static final MechanismLigament2d kElevatorTower;

    static
    {
        kElevatorCarriage = Constants.sideRobotView.getRoot("ElevatorCarriage",
                ArmConstants.armConfig.kArmLength,
                ElevatorConstants.elevatorConfig.kStartingHeightSim.in(
                        Meters));
        kArmMech = kElevatorCarriage.append(
                new MechanismLigament2d(
                        "Arm",
                        ArmConstants.armConfig.kArmLength,
                        ArmConstants.armConfig.kArmStartingAngle.in(Degrees),
                        6,
                        new Color8Bit(Color.kYellow)));
        kElevatorTower = kElevatorCarriage.append(new MechanismLigament2d(
                "Elevator",
                ElevatorConstants.elevatorConfig.kStartingHeightSim.in(Meters),
                -90,
                6,
                new Color8Bit(Color.kRed)));
    }

    public static final class ArmConstants {
        public static final double armSpeed = 0.15;
        public static final double armSetpoint = 5;

        public static final ManipArmConstants armConfig =
                new ManipArmConstants(
                        DCMotor.getNEO(1),
                        2.026,
                        0,
                        0.092372,
                        0.012394,
                        0.019009,
                        0.12269,
                        0.0042372,
                        140,
                        3,
                        17,
                        0,
                        -90,
                        90,
                        true,
                        .5,
                        0,
                        0.01,
                        40,
                        20,
                        40,
                        true
                );
    }

    public static final class ElevatorConstants {
        public static final double elevatorSpeed = .3;
        public static final double elevatorSetpoint = 10;

        public static final ManipElevatorConstants elevatorConfig =
                new ManipElevatorConstants(
                        DCMotor.getNEO(2),
                        28.475,
                        0,
                        1.2353,
                        0.08,
                        23.47,
                        0.45118,
                        0.044757,
                        60,
                        2,
                        6,
                        0,
                        72,
                        0,
                        false,
                        0.2,
                        40,
                        10,
                        20,
                        0,
                        true
                );
    }

    public static final class IntakeShooterConstants {
        public static final double intakeSpeed = .3;
        public static final double shootSpeed = .5;
        public static final double velocitySetpoint = 100;
    }

}
