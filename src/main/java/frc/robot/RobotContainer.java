// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.io.File;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Constants.OperatorConstants;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.SwerveSubsystem;

public class RobotContainer {
  private final SwerveSubsystem drivebase = new SwerveSubsystem(
      new File(Filesystem.getDeployDirectory(), "swerve/neo"));
  private final CommandXboxController driverXbox = new CommandXboxController(0);
  private final ArmSubsystem arm = new ArmSubsystem();
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
        () -> MathUtil.clamp(MathUtil.applyDeadband(driverXbox.getLeftY(), OperatorConstants.LEFT_Y_DEADBAND), -1,
            1),
        () -> MathUtil.clamp(MathUtil.applyDeadband(driverXbox.getLeftX(), OperatorConstants.LEFT_X_DEADBAND), -1,
            1),
        () -> -driverXbox.getRightX());

    drivebase.setDefaultCommand(driveFieldOrientedAnglularVelocity);
    // drivebase.setDefaultCommand(closedAbsoluteDrive);
  }
  private void configureBindings() {
    driverXbox.a().onTrue(new InstantCommand(drivebase::lock, drivebase));
    driverXbox.x().onTrue(new InstantCommand(drivebase::zeroGyro, drivebase).ignoringDisable(true));
    // driverXbox.povDown().onTrue(drivebase.driveToPose(new Pose2d(1.89, 7.67, new Rotation2d(Math.toRadians(90)))).
    //                             andThen(arm.scoreAmp()));
    // driverXbox.povUp().onTrue(drivebase.driveToPose(new Pose2d(15.47, 0.89, new Rotation2d(Math.toRadians(-60)))).
    //                           andThen(arm.intake()).
    //                           andThen(new WaitCommand(5)).
    //                           andThen(arm.stopEverything()));


    driverXbox.povDown().whileTrue(arm.intake()).onFalse(Commands.run(arm::stopEverything));
        driverXbox.povLeft().onTrue(arm.moveElevatorTo(-44));
    driverXbox.povLeft().onTrue(arm.moveWristTo(-18));
    driverXbox.povUp().onTrue(arm.outtake().andThen(new WaitCommand(2)).andThen(arm.stopEverything())).onFalse(Commands.run(arm::stopEverything));


    driverXbox.b().onTrue(arm.moveWristTo(0).andThen(arm.moveElevatorTo(0)));
    driverXbox.rightTrigger().whileTrue(arm.dumshoot()).onFalse(Commands.run(arm::stopEverything));
    driverXbox.leftTrigger().whileTrue(arm.dumamp()).onFalse(Commands.run(arm::stopEverything));
    driverXbox.leftBumper().whileTrue(arm.goSlowDown());
    driverXbox.rightBumper().whileTrue(arm.goSlowUp());

        driverXbox.y().whileTrue(arm.dumbExtendElevator()).onFalse(Commands.run(arm::stopEverything));
    driverXbox.a().whileTrue(arm.dumbRetractElevator()).onFalse(Commands.run(arm::stopEverything));

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
