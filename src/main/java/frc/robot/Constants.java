package frc.robot;

import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.wpilibj.smartdashboard.Mechanism2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismLigament2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismRoot2d;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj.util.Color8Bit;
import maniplib.utils.ManipArmConstants;
import maniplib.utils.ManipElevatorConstants;

import static edu.wpi.first.units.Units.*;
import static frc.robot.Constants.ArmConstants.armConfig;

public class Constants {
    public static final class ArmConstants {
        public static final double armSpeed = 0.15;
        public static final double armSetpoint = 5;

        public static final ManipArmConstants armConfig =
                new ManipArmConstants(
                        DCMotor.getNEO(1),
                        3,
                        0,
                        0,
                        0,
                        0,
                        0,
                        0,
                        140,
                        3,
                        17,
                        0,
                        -90,
                        90,
                        false,
                        .5,
                        0,
                        0.01,
                        40,
                        90,
                        180,
                        true
                );
    }

    public static final class ElevatorConstants {
        public static final double elevatorSpeed = .3;
        public static final double elevatorSetpoint = 10;

        public static final ManipElevatorConstants elevatorConfig =
                new ManipElevatorConstants(
                        DCMotor.getNEO(1),
                        26.722,
                        0,
                        1.6047,
                        0.01964,
                        3.894,
                        0.173,
                        0.91274,
                        50,
                        2,
                        8,
                        0,
                        72,
                        0,
                        0.1,
                        40,
                        4,
                        8,
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
