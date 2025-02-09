// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.subsystems.ElevatorSubsystem;
import frc.robot.subsystems.IntakeShooterSubsystem;
import maniplib.ManipArm;
import maniplib.motors.ManipSparkMax;

public class RobotContainer {

    // Define the ManipLib customized subsystems.
    private final ManipSparkMax armMotor = new ManipSparkMax(1);
    private final ManipArm arm = new ManipArm(armMotor, Constants.ArmConstants.armConfig);

    private final ElevatorSubsystem elevatorSubsystem = new ElevatorSubsystem();
    private final IntakeShooterSubsystem intakeShooterSubsystem = new IntakeShooterSubsystem();

    // We only need operator controller for this project
    private final CommandXboxController operatorController = new CommandXboxController(0);

    public RobotContainer() {

        DriverStation.silenceJoystickConnectionWarning(true);

        arm.setDefaultCommand(arm.defaultCommandWithOverride(0));

        configureBindings();
    }

    private void configureBindings() {

        // Can also make actual subsystem classes with something like armSubsystem.intake() to clean up RobotContainer
        operatorController.a().whileTrue(arm.runEnd(() -> arm.runArm(.15), arm::stopArm));
        operatorController.b().whileTrue(arm.runEnd(() -> arm.runArm(-.15), arm::stopArm));

        operatorController.back().onTrue(arm.overrideDefaultCommand());

        operatorController.y().whileTrue(arm.setGoal(-75));
        operatorController.x().whileTrue(arm.setGoal(75));

        // Basically an E-Stop
        operatorController.start().onTrue(arm.runSysIdRoutine());
    }

    public Command getAutonomousCommand() {
        return Commands.print("No autonomous command configured");
    }
}
