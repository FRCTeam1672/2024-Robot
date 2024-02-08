// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.io.File;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Constants.OperatorConstants;
import frc.robot.subsystems.SwerveSubsystem;

public class RobotContainer {
  private final SwerveSubsystem drivebase = new SwerveSubsystem(
      new File(Filesystem.getDeployDirectory(), "swerve/neo"));
  CommandXboxController driverXbox = new CommandXboxController(0);

  /**
   * The container for the robot. Contains subsystems, OI devices, and commands.
   */
  public RobotContainer() {
    // Configure the trigger bindings
    configureBindings();
    // Applies deadbands and inverts controls because joysticks
    // are back-right positive while robot
    // controls are front-left positive
    // left stick controls translation
    // right stick controls the angular velocity of the robot
    Command driveFieldOrientedAnglularVelocity = drivebase.driveCommand(
        () -> MathUtil.clamp(MathUtil.applyDeadband(-driverXbox.getLeftY(), OperatorConstants.LEFT_Y_DEADBAND), -1,
            1),
        () -> MathUtil.clamp(MathUtil.applyDeadband(-driverXbox.getLeftX(), OperatorConstants.LEFT_X_DEADBAND), -1,
            1),
        () -> -driverXbox.getRightX());

    drivebase.setDefaultCommand(driveFieldOrientedAnglularVelocity);
    // drivebase.setDefaultCommand(closedAbsoluteDrive);
  }

  /**
   * Use this method to define your trigger->command mappings. Triggers can be
   * created via the
   * {@link Trigger#Trigger(java.util.function.BooleanSupplier)} constructor with
   * an arbitrary predicate, or via the
   * named factories in
   * {@link edu.wpi.first.wpilibj2.command.button.CommandGenericHID}'s subclasses
   * for
   * {@link CommandXboxController
   * Xbox}/{@link edu.wpi.first.wpilibj2.command.button.CommandPS4Controller PS4}
   * controllers or {@link edu.wpi.first.wpilibj2.command.button.CommandJoystick
   * Flight joysticks}.
   */
  private void configureBindings() {
    // driverXbox.x().onTrue(new InstantCommand(drivebase::addFakeVisionReading));
    driverXbox.a().onTrue(new InstantCommand(drivebase::lock, drivebase));
    driverXbox.x().onTrue(new InstantCommand(drivebase::zeroGyro, drivebase).ignoringDisable(true));
    driverXbox.b().onTrue(new InstantCommand(() -> {
      drivebase.resetOdometry(new Pose2d());
    }, drivebase));
    // SmartDashboard.putData("Push Encoder Values", new InstantCommand(() -> {
    // drivebase.pushOffsetsToControllers();
    // }));
    SmartDashboard.putData("Zero Gyro", new InstantCommand(drivebase::zeroGyro).ignoringDisable(true));

    // new JoystickButton(driverXbox, driverXbox).onTrue((new
    // InstantCommand(drivebase::zeroGyro)));
    // new JoystickButton(driverXbox, 3).onTrue(new
    // InstantCommand(drivebase::addFakeVisionReading));
    // new JoystickButton(driverXbox, 3).whileTrue(new RepeatCommand(new
    // InstantCommand(drivebase::lock, drivebase)));
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    // An example command will be run in autonomous
    return Commands.sequence(
      drivebase.getAutonomousCommand("Figure 8 Notes", true),
      drivebase.getAutonomousCommand("Figure 8 Notes", false)
    );
  }

  public void setDriveMode() {
    // drivebase.setDefaultCommand();
  }

  // 100% goon activated
  public void setMotorBrake(boolean brake) {
    drivebase.setMotorBrake(brake);
  }
}
