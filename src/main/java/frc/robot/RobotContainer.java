// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.io.File;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Constants.OperatorConstants;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.LEDSubsytem;
import frc.robot.subsystems.SwerveSubsystem;
import frc.robot.subsystems.VisionSubsystem;

public class RobotContainer {
  private final SwerveSubsystem drivebase = new SwerveSubsystem(
      new File(Filesystem.getDeployDirectory(), "swerve/neo"));
  private final CommandXboxController driverXbox = new CommandXboxController(0);
  private final CommandXboxController oppsController = new CommandXboxController(1);
  private final ArmSubsystem arm = new ArmSubsystem();
  private final LEDSubsytem ledSubsytem = new LEDSubsytem();
  private final VisionSubsystem visionSubsystem = new VisionSubsystem();
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

    //DoubleSupplier 


    Command driveFieldOrientedAnglularVelocity = drivebase.driveCommand(
        () -> {
          return MathUtil.clamp(MathUtil.applyDeadband(-driverXbox.getLeftY(), OperatorConstants.LEFT_Y_DEADBAND), -1,1);
        },
        () -> {
          return MathUtil.clamp(MathUtil.applyDeadband(-driverXbox.getLeftX(), OperatorConstants.LEFT_X_DEADBAND), -1,1);
        },
        () -> {
          return driverXbox.getRightX();
        }, 
        () -> driverXbox.leftBumper().getAsBoolean());

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


    oppsController.povDown().whileTrue(arm.intake()).onFalse(Commands.run(arm::stopEverything));

    //amp position
    oppsController.povLeft().onTrue(Commands.parallel(arm.moveElevatorTo(Constants.Aim.ELEVATOR_HEIGHT_AMP), arm.moveWristTo(Constants.Aim.WRIST_ANGLE_AMP)));
    
    oppsController.povRight().onTrue(arm.shoot().andThen(new WaitCommand(2)).andThen(arm.stopEverything()));
    oppsController.povUp().onTrue(arm.outtake().andThen(new WaitCommand(2)).andThen(arm.stopEverything())).onFalse(Commands.run(arm::stopEverything));

    //retract everything but keep hovering
    oppsController.b().onTrue(arm.moveWristTo(Constants.Aim.HOME_POSITION).andThen(arm.moveElevatorTo(Constants.Aim.HOME_POSITION)));
    //move to source position
    oppsController.y().onTrue(arm.moveElevatorTo(Constants.Aim.ELEVATOR_HEIGHT_SOURCE));
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
  public VisionSubsystem getVisionSubsystem() {
    return visionSubsystem;
  }
  public SwerveSubsystem getDrivebase() {
    return drivebase;
  }
}
