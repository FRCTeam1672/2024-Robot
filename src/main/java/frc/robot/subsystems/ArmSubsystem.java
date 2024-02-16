// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkMax;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.Constants;

public class ArmSubsystem extends SubsystemBase {
  private CANSparkMax lElevator = new CANSparkMax(31, MotorType.kBrushless);
  private CANSparkMax rElevator = new CANSparkMax(32, MotorType.kBrushless);
  private CANSparkMax lShooter = new CANSparkMax(41, MotorType.kBrushless);
  private CANSparkMax rShooter = new CANSparkMax(42, MotorType.kBrushless);
  private CANSparkMax lFeeder = new CANSparkMax(43, MotorType.kBrushless);
  private CANSparkMax rFeeder = new CANSparkMax(44, MotorType.kBrushless);
  private CANSparkMax wrist = new CANSparkMax(50, MotorType.kBrushless);

  private DigitalInput limitSwitch = new DigitalInput(0);

  private PIDController wristPidController = new PIDController(0.01, 0, 0);
  private PIDController elevatorPidController = new PIDController(0.01, 0, 0);

  /** Creates a new ArmSubsystem. */
  public ArmSubsystem() {
    rElevator.follow(lElevator, true);
    rShooter.follow(lShooter, true);
    rFeeder.follow(lFeeder, true);

  }

  @Override
  public void periodic() {
    SmartDashboard.putNumber("LShooter Velocity", lShooter.getEncoder().getVelocity());
    SmartDashboard.putNumber("RShooter Velocity", rShooter.getEncoder().getVelocity());
    SmartDashboard.putNumber("LIntake Velocity", lFeeder.getEncoder().getVelocity());
    SmartDashboard.putNumber("RIntake Velocity", rFeeder.getEncoder().getVelocity());
    SmartDashboard.putNumber("Wrist Angle", wrist.getEncoder().getPosition());
    SmartDashboard.putNumber("rElevator Height", rElevator.getEncoder().getPosition());
    SmartDashboard.putNumber("lElevator Height", lElevator.getEncoder().getPosition());
  }

  //stop everything
  public Command stopEverything() {
    return Commands.runOnce(() -> {
      lElevator.stopMotor();
      rElevator.stopMotor();
      lShooter.stopMotor();
      rShooter.stopMotor();
      lFeeder.stopMotor();
      rFeeder.stopMotor();
      wrist.stopMotor();
    });
  }
  public void stopEverythingMethod() {
    lElevator.stopMotor();
      rElevator.stopMotor();
      lShooter.stopMotor();
      rShooter.stopMotor();
      lFeeder.stopMotor();
      rFeeder.stopMotor();
      wrist.stopMotor();
  }

  public boolean isHomed() {
    return limitSwitch.get();
  }

  public Command homeElevator() {
    return Commands.run(() -> {
      lElevator.set(Constants.Elevator.HOME_SPEED);
    }).until(() -> {
      return isHomed();
    });
  }

  public Command goSlowUp() {
    return Commands.run(() -> {
      wrist.set(Constants.Aim.SLOW_PRECISION_SPEED);
    }).handleInterrupt(() -> {stopEverythingMethod();});
  }

  public Command goSlowDown() {
    return Commands.run(() -> {
      wrist.set(-Constants.Aim.SLOW_PRECISION_SPEED);
    }).handleInterrupt(() -> {stopEverythingMethod();});
  }

  public Command intake() {
    return Commands.run(() -> {
      lShooter.set(Constants.Intake.INTAKE_SPEED); // negative is clockwise
      lFeeder.set(Constants.Intake.INTAKE_SPEED);
    }).handleInterrupt(this::stopEverythingMethod);
  }

  public Command shoot() {
    return Commands.run(() -> {
      // spin outer wheels to 100 power
      lShooter.set(Constants.Intake.SHOOT_SPEED);
    }).until(() -> {
      return lShooter.getEncoder().getVelocity() < Constants.Intake.SHOOT_VELOCITY;
    }).andThen(Commands.runOnce(() -> {
      lFeeder.set(Constants.Intake.SHOOT_SPEED);
    }))
    .andThen(new WaitCommand(2))
    .andThen(() -> {
      System.out.println("we are done with the command");
      stopEverythingMethod();
    });
  }

  public Command dumshoot() {
    return Commands.run(() -> {
      lShooter.set(Constants.Intake.SHOOT_SPEED);
    }).handleInterrupt(() -> {stopEverythingMethod();});
  }

  public Command dumamp() {
    return Commands.run(() -> {
      lShooter.set(Constants.Intake.AMP_OUTTAKE_SPEED);
    }).handleInterrupt(() -> {stopEverythingMethod();});
  }
  
  public Command dumbExtendElevator() {
    return Commands.run(() -> {
      lElevator.set(-0.1);
    }).handleInterrupt(() -> {stopEverythingMethod();});
  }
  public Command dumbRetractElevator() {
    return Commands.run(() -> {
      lElevator.set(0.1);
    }).handleInterrupt(() -> {stopEverythingMethod();});
  }

  public Command outtake() {
    return Commands.run(() -> {
      // spin outer wheels to 10 power
      lShooter.set(Constants.Intake.AMP_OUTTAKE_SPEED);
    }).until(() -> {
      return lShooter.getEncoder().getVelocity() < Constants.Intake.OUTTAKE_VELOCITY;
      // TODO change this value
    }).andThen(Commands.run(() -> {
      lFeeder.set(Constants.Intake.AMP_OUTTAKE_SPEED);
    }).handleInterrupt(this::stopEverythingMethod));
  }
  
  public Command moveElevatorTo(int pos) {
    elevatorPidController.setSetpoint(pos);
    return Commands.run(() -> {
      // negative for clockwise
      lElevator.set(MathUtil.clamp(elevatorPidController.calculate(lElevator.getEncoder().getPosition()), Constants.Elevator.HOME_SPEED, -Constants.Elevator.HOME_SPEED));
    }).until(() -> {
      return elevatorPidController.atSetpoint();
    }).andThen(stopEverything());
  }
  
  /**
   * Moves the wrist to a position.
   * @param pos the position to go to
   * @return the command to move the wrist
   */
  public Command moveWristTo(int pos) {
    wristPidController.setSetpoint(pos);

    return Commands.run(() -> {
      wrist.set(MathUtil.clamp(wristPidController.calculate(wrist.getEncoder().getPosition()), -Constants.Aim.AMP_AIM_SPEED, Constants.Aim.AMP_AIM_SPEED));
    }).until(() -> {
      return wristPidController.atSetpoint();
    }).andThen(Commands.runOnce(() -> {
      wrist.stopMotor();
    }));
  }

  public Command scoreAmp() {
    return  moveWristTo(Constants.Aim.WRIST_ANGLE_AMP).
            alongWith(moveElevatorTo(Constants.Aim.ELEVATOR_HEIGHT_AMP)).
            andThen(outtake()).
            andThen(new WaitCommand(1)).
            andThen(stopEverything()).
            andThen(homeElevator()).
            alongWith(moveWristTo(0));
  }

  public Command scoreSpeaker() {
    return  moveWristTo(Constants.Aim.WRIST_ANGLE_SPAKER).
            alongWith(moveElevatorTo(Constants.Aim.ELEVATOR_HEIGHT_SPEAKER)).
            andThen(shoot()).
            andThen(new WaitCommand(2)).
            andThen(stopEverything()).
            andThen(homeElevator()).
            alongWith(moveWristTo(0));
  }
  
}
