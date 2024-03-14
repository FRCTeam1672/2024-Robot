// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;
import java.io.File;
import java.util.Optional;

import com.pathplanner.lib.auto.NamedCommands;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ProxyCommand;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.button.CommandJoystick;
import edu.wpi.first.wpilibj2.command.button.CommandPS5Controller;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Constants.OperatorConstants;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.ClimbSubsystem;
import frc.robot.subsystems.LEDSubsytem;
import frc.robot.subsystems.SwerveSubsystem;
import frc.robot.subsystems.VisionSubsystem;

//37, 97, and 98 are climb
//uncomment eventually

public class RobotContainer {
  private final SwerveSubsystem drivebase = new SwerveSubsystem(
      new File(Filesystem.getDeployDirectory(), "swerve/neo"));
  private final CommandJoystick driverPS5 = new CommandJoystick(0);
  private final CommandPS5Controller oppsController = new CommandPS5Controller(1);
  private final ArmSubsystem arm = new ArmSubsystem();
  private final LEDSubsytem ledSubsytem = new LEDSubsytem();
  private final VisionSubsystem visionSubsystem = new VisionSubsystem();
  private final ClimbSubsystem climb = new ClimbSubsystem();
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
    NamedCommands.registerCommand("scoreAmp", new ProxyCommand(arm.goToAmpPosition()));


    Command driveFieldOrientedAnglularVelocity = drivebase.driveCommand(
        () -> {
          return MathUtil.clamp(MathUtil.applyDeadband(driverPS5.getY(), OperatorConstants.LEFT_Y_DEADBAND), -1,1);
          // if(DriverStation.getAlliance().isEmpty()) return 0;
          // else if(DriverStation.getAlliance().get() == Alliance.Blue) return MathUtil.clamp(MathUtil.applyDeadband(-driverXbox.getLeftY(), OperatorConstants.LEFT_Y_DEADBAND), -1,1);
          // else return MathUtil.clamp(MathUtil.applyDeadband(driverXbox.getLeftY(), OperatorConstants.LEFT_Y_DEADBAND), -1,1);
        },
        () -> {  

        return MathUtil.clamp(MathUtil.applyDeadband(driverPS5.getX(), OperatorConstants.LEFT_X_DEADBAND), -1,1);
          // if(DriverStation.getAlliance().isEmpty()) return 0;
          // else if(DriverStation.getAlliance().get() == Alliance.Blue) return MathUtil.clamp(MathUtil.applyDeadband(-driverXbox.getLeftX(), OperatorConstants.LEFT_X_DEADBAND), -1,1);
          // else return MathUtil.clamp(MathUtil.applyDeadband(driverXbox.getLeftX(), OperatorConstants.LEFT_X_DEADBAND), -1,1);
          
        },
        () -> {
          return driverPS5.getZ();
        }, 
        () -> driverPS5.button(1).getAsBoolean());

    drivebase.setDefaultCommand(driveFieldOrientedAnglularVelocity);
    // drivebase.setDefaultCommand(closedAbsoluteDrive);  
  } 
  public static final Pose2d convertToRedSide(Pose2d pose) {
    Optional<Alliance> alliance = DriverStation.getAlliance();

    return pose;
}
  private void configureBindings() {

    // driverPS5.R1().onTrue(new InstantCommand(drivebase::pointModulesForward , drivebase));
    // // driverXbox.cross().onTrue(new InstantCommand(drivebase::lock, drivebase));
    // driverPS5.square().onTrue(new InstantCommand(drivebase::zeroGyro, drivebase).ignoringDisable(true));
    // driverPS5.povUp().onTrue(drivebase.driveToPose(new Pose2d(14, 5.48, new Rotation2d(Math.toRadians(0)))));
    driverPS5.button(4).whileTrue(drivebase.pathFindAndAutoCommand("Source Align").andThen(arm.moveElevatorTo(Constants.Aim.ELEVATOR_HEIGHT_SOURCE)));
    driverPS5.button(3).whileTrue(drivebase.pathFindAndAutoCommand("AmpAlign").
        andThen(arm.goToAmpPosition()
        .andThen(
          Commands.waitUntil(() -> {
            return arm.shouldMoveWristJoint() && arm.isAtPosition();
          })
          .andThen(arm.outtake()).withTimeout(1.5).andThen(arm.homeEverything())
          .handleInterrupt(arm::stopEverythingMethod)
        ))
        .handleInterrupt(arm::stopEverythingMethod)
      );  

    // driverXbox.b().onTrue(
    //   drivebase.getAutonomousCommand("Source", false).
    //   andThen(getAutonomousCommand()).andThen(Commands.runOnce(drivebase::pointModulesForward, drivebase))
    // );
    // driverXbox.povDown().onTrue(drivebase.driveToPose(new Pose2d(1.89, 7.67, new Rotation2d(Math.toRadians(90)))).
    //                             andThen(arm.scoreAmp()));
    // driverXbox.povUp().onTrue(drivebase.driveToPose(new Pose2d(15.47, 0.89, new Rotation2d(Math.toRadians(-60)))).
    //                           andThen(arm.intake()).
    //                           andThen(new WaitCommand(5)).
    //                           andThen(arm.stopEverything()));


    oppsController.povDown().whileTrue(arm.intake()).onFalse(Commands.run(arm::stopEverything));

    //amp position
    //oppsController.x().onTrue(arm.moveWristTo(Constants.Aim.WRIST_ANGLE_AMP));
    //oppsController.x().onTrue(arm.goToAmpPosition());

    oppsController.square().onTrue(arm.moveElevatorTo(-90));
    
    oppsController.cross().onTrue(arm.shoot().andThen(new WaitCommand(2)).andThen(arm.stopEverything()));
    oppsController.povUp().whileTrue(arm.outtake()).onFalse(Commands.run(arm::stopEverything));

    //retract everything but keep hovering
    oppsController.circle().onTrue(arm.homeEverything());
    //move to source position
    oppsController.triangle().onTrue(arm.moveElevatorTo(Constants.Aim.ELEVATOR_HEIGHT_SOURCE));
    oppsController.L1().whileTrue(climb.goDown().handleInterrupt(climb::stop));
    oppsController.R1().whileTrue(climb.goUp().handleInterrupt(climb::stop));
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    // An example command will be run in autonomousa
    // return new InstantCommand(drivebase::pointModulesForward , drivebase).andThen(Commands.runOnce(() -> drivebase.drive(new Translation2d(3 , 0), 0, false),drivebase)
    // .andThen(Commands.waitSeconds(2)).andThen(() -> drivebase.drive(new Translation2d(0, 0), 0, false)));
    return drivebase.getAutonomousCommand("AmpAmpApproach", false)
      .andThen(Commands.waitUntil(() -> {
        return arm.shouldMoveWristJoint() && arm.isAtPosition();
      })
      .andThen(arm.outtake()
        .withTimeout(0.5)
        .andThen(
          Commands.parallel(
              drivebase.getAutonomousCommand("AmpLeave", false),
              arm.homeEverything()
          )
        )
      
    ));
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
