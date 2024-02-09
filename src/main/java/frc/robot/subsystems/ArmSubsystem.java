// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkMax;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.Constants;

public class ArmSubsystem extends SubsystemBase {
  private CANSparkMax lElevator = new CANSparkMax(12832, MotorType.kBrushless);
  private CANSparkMax rElevator = new CANSparkMax(1000, MotorType.kBrushless);
  private CANSparkMax lShooter = new CANSparkMax(1001, MotorType.kBrushless);
  private CANSparkMax rShooter = new CANSparkMax(1002, MotorType.kBrushless);
  private CANSparkMax lFeeder = new CANSparkMax(100202, MotorType.kBrushless);
  private CANSparkMax rFeeder = new CANSparkMax(100102, MotorType.kBrushless);
  private CANSparkMax wrist = new CANSparkMax(100702, MotorType.kBrushless);

  private DigitalInput limitSwitch = new DigitalInput(0);

  private PIDController wristPidController = new PIDController(0.01, 0, 0);
  private PIDController elevatorPidController = new PIDController(0.01, 0, 0);

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

  public boolean isHomed() {
    return limitSwitch.get();
  }

  public Command homeElevator() {
    return Commands.run(() -> {
      lElevator.set(Constants.Elevator.HOMESPEED);
    }).until(() -> {
      return isHomed();
    });
  }

  

  public Command intake() {
    return Commands.run(() -> {
      lShooter.set(Constants.Intake.INTAKE_SPEED); // negative is clockwise
      lFeeder.set(Constants.Intake.INTAKE_SPEED);
    });
  }

  public Command shoot() {
    return Commands.run(() -> {
      // spin outer wheels to 100 power
      lShooter.set(Constants.Intake.SHOOT_SPEED);
    }).until(() -> {
      return lShooter.getEncoder().getVelocity() < Constants.Intake.SHOOT_VELOCITY;
      // TODO get this actual max value of motors
    }).andThen(Commands.run(() -> {
      lFeeder.set(Constants.Intake.SHOOT_SPEED);
    }));
  }

  public Command outtake() {
    return Commands.run(() -> {
      // spin outer wheels to 10 power
      lShooter.set(Constants.Intake.AMP_OUTTAKE_SPEED);
      // TODO get the actual values
    }).until(() -> {
      return lShooter.getEncoder().getVelocity() < Constants.Intake.OUTTAKE_VELOCITY;
      // TODO change this value
    }).andThen(Commands.run(() -> {
      lFeeder.set(Constants.Intake.AMP_OUTTAKE_SPEED);
    }));
  }
  
  public Command moveElevatorTo(int pos) {
    elevatorPidController.setSetpoint(pos);
    return Commands.run(() -> {
      // negative for clockwise
      lElevator.set(MathUtil.clamp(elevatorPidController.calculate(lElevator.getEncoder().getPosition()), -0.1, 0.1)); //TODO make these constants
      //TODO do MathUtil.clamp() for all PID controllers
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
    //TODO put postion
    wristPidController.setSetpoint(pos);

    return Commands.run(() -> {
      wrist.set(wristPidController.calculate(wrist.getEncoder().getPosition()));
    }).until(() -> {
      return wristPidController.atSetpoint();
    }).andThen(Commands.runOnce(() -> {
      wrist.stopMotor();
    }));
  }

  public Command scoreAmp() {
    return  moveWristTo(Constants.Aim.WristAngleAmp).
            alongWith(moveElevatorTo(Constants.Aim.ElevatorHeightAmp)).
            andThen(outtake()).
            andThen(new WaitCommand(2)).
            andThen(stopEverything()).
            andThen(homeElevator()).
            alongWith(moveWristTo(0));
  }

  public Command scoreSpeaker() {
    return  moveWristTo(Constants.Aim.WristAngleSpeaker).
            alongWith(moveElevatorTo(Constants.Aim.ElevatorHeightSpeaker)).
            andThen(shoot()).
            andThen(new WaitCommand(2)).
            andThen(stopEverything()).
            andThen(homeElevator()).
            alongWith(moveWristTo(0));
  }
  
  /** Creates a new ArmSubsystem. */
  public ArmSubsystem() {
    rElevator.follow(lElevator);
    rElevator.setInverted(true);
    rShooter.follow(lShooter);
    rShooter.setInverted(true);
    rFeeder.follow(lFeeder);
    rFeeder.setInverted(true);

  }

  @Override
  public void periodic() {

  }
}
