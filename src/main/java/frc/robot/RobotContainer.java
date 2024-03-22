// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.io.File;
import java.util.Optional;

import javax.sound.midi.Soundbank;

import com.pathplanner.lib.auto.NamedCommands;
import com.pathplanner.lib.commands.FollowPathCommand;
import com.pathplanner.lib.commands.PathfindingCommand;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.ADXL345_I2C.AllAxes;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
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
  private final CommandPS5Controller driverPS5 = new CommandPS5Controller(0);
  private final CommandPS5Controller oppsController = new CommandPS5Controller(1);
  private final ArmSubsystem arm = new ArmSubsystem();
  private final LEDSubsytem ledSubsytem = new LEDSubsytem();
  private final VisionSubsystem visionSubsystem = new VisionSubsystem();
  private final ClimbSubsystem climb = new ClimbSubsystem();

  private final SendableChooser<Command> stageOneAuto = new SendableChooser<>();
  private final SendableChooser<Command> stageTwoAuto = new SendableChooser<>();

  /**
   * The container for the robot. Contains subsystems, OI devices, and commands.
   */
  public RobotContainer() {
    // Configure the trigger bindings
    NamedCommands.registerCommand("scoreAmp", new ProxyCommand(arm.goToAmpPosition()));
    PathfindingCommand.warmupCommand().schedule();
    FollowPathCommand.warmupCommand().schedule();
    configureBindings();

    stageOneAuto.addOption("Amp Side", drivebase.getAutonomousCommand("AmpAmpApproach", false).andThen(getScoreCommand()).asProxy());
    stageOneAuto.addOption("Center Side", drivebase.getAutonomousCommand("CenterAmpApproach", false).andThen(getScoreCommand()).asProxy());
    stageOneAuto.addOption("Source Side", drivebase.getAutonomousCommand("SourceAmpApproach", false).andThen(getScoreCommand()).asProxy());
    stageOneAuto.setDefaultOption("Nothing", Commands.none());

    stageTwoAuto.addOption("Leave", drivebase.getAutonomousCommand("AmpLeave", false).asProxy());
    stageTwoAuto.addOption("Eliminate the Notes", drivebase.getAutonomousCommand("AmpNotes", false).asProxy());
    stageTwoAuto.setDefaultOption("Nothing", Commands.none());

    SmartDashboard.putData("Stage One Auto", stageOneAuto);
    SmartDashboard.putData("Stage Two Auto", stageTwoAuto);

    Command driveFieldOrientedAnglularVelocity = drivebase.driveCommand(
        () -> {
          if(DriverStation.getAlliance().isEmpty()) return 0;
          if(DriverStation.getAlliance().get() == Alliance.Red) return MathUtil.clamp(MathUtil.applyDeadband(driverPS5.getLeftY(), OperatorConstants.LEFT_Y_DEADBAND), -1, 1);
          else return MathUtil.clamp(MathUtil.applyDeadband(-driverPS5.getLeftY(), OperatorConstants.LEFT_Y_DEADBAND), -1, 1);
        },
        () -> {
          if(DriverStation.getAlliance().isEmpty()) return 0;
          if(DriverStation.getAlliance().get() == Alliance.Red) return MathUtil.clamp(MathUtil.applyDeadband(driverPS5.getLeftX(), OperatorConstants.LEFT_X_DEADBAND), -1, 1);
          else return MathUtil.clamp(MathUtil.applyDeadband(-driverPS5.getLeftX(), OperatorConstants.LEFT_X_DEADBAND), -1, 1);

          // if(DriverStation.getAlliance().isEmpty()) return 0;
          // else if(DriverStation.getAlliance().get() == Alliance.Blue) return
          // MathUtil.clamp(MathUtil.applyDeadband(-driverXbox.getLeftX(),
          // OperatorConstants.LEFT_X_DEADBAND), -1,1);
          // else return MathUtil.clamp(MathUtil.applyDeadband(driverXbox.getLeftX(),
          // OperatorConstants.LEFT_X_DEADBAND), -1,1);

        },
        () -> {
          return driverPS5.getRightX();
        },
        () -> driverPS5.R1().getAsBoolean());

    drivebase.setDefaultCommand(driveFieldOrientedAnglularVelocity);
    // drivebase.setDefaultCommand(closedAbsoluteDrive);
  }

  private void configureBindings() {

    // driverPS5.R1().onTrue(new InstantCommand(drivebase::pointModulesForward ,
    // drivebase));
    // // driverXbox.cross().onTrue(new InstantCommand(drivebase::lock, drivebase));
    // driverPS5.square().onTrue(new InstantCommand(drivebase::zeroGyro,
    // drivebase).ignoringDisable(true));
    // driverPS5.povUp().onTrue(drivebase.driveToPose(new Pose2d(14, 5.48, new
    // Rotation2d(Math.toRadians(0)))));
    driverPS5.L2().whileTrue(drivebase.pathFindAndAutoCommand("Source Align").andThen(
        arm.moveElevatorTo(Constants.Aim.ELEVATOR_HEIGHT_SOURCE)
            .andThen(Commands.waitUntil(() -> {
              return arm.isAtPosition();
            }).andThen(arm::stopEverything).handleInterrupt(arm::stopEverything))
            )
        );
    driverPS5.R2().whileTrue(drivebase.pathFindAndAutoCommand("AmpAlign").andThen(arm.goToAmpPosition()
        .andThen(
            Commands.waitUntil(() -> {
              return arm.shouldMoveWristJoint() && arm.isAtPosition();
            })
                .andThen(arm.outtake().withTimeout(1)).andThen(arm.homeEverything())
                .handleInterrupt(arm::stopEverythingMethod)))
        .handleInterrupt(arm::stopEverythingMethod));
    driverPS5.circle().onTrue(arm.homeEverything());
    driverPS5.povDown().whileTrue(arm.intake()).onFalse(Commands.run(arm::stopEverything));

    oppsController.povDown().whileTrue(arm.intake()).onFalse(Commands.run(arm::stopEverything));

    oppsController.square().onTrue(arm.goToAmpPosition());

    oppsController.cross().onTrue(arm.shoot().andThen(new WaitCommand(2)).andThen(arm.stopEverything()));
    oppsController.povUp().whileTrue(arm.outtake()).onFalse(Commands.run(arm::stopEverything));

    // retract everything but keep hovering
    oppsController.circle().onTrue(arm.homeEverything());
    // move to source position
    oppsController.triangle().onTrue(arm.moveElevatorTo(Constants.Aim.ELEVATOR_HEIGHT_SOURCE));
    oppsController.L1().whileTrue(climb.goDown().handleInterrupt(climb::stop));
    oppsController.R1().whileTrue(climb.goUp().handleInterrupt(climb::stop));
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */

  public Command getScoreCommand() {
    return Commands.waitUntil(() -> {
          return arm.shouldMoveWristJoint() && arm.isAtPosition();
        }).andThen(
          arm.outtake().withTimeout(0.5)
        );
  }
  public Command getAutonomousCommand() {
    return stageOneAuto.getSelected()
                .andThen(
                    Commands.parallel(
                        stageTwoAuto.getSelected(),
                        arm.homeEverything())

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
