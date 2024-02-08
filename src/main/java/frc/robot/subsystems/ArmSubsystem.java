// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkMax;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class ArmSubsystem extends SubsystemBase {
  private CANSparkMax lElevator = new CANSparkMax(12832, MotorType.kBrushless);
  private CANSparkMax rElevator = new CANSparkMax(1000, MotorType.kBrushless);
  private CANSparkMax lShooter = new CANSparkMax(1001, MotorType.kBrushless);
  private CANSparkMax rShooter = new CANSparkMax(1002, MotorType.kBrushless);
  private CANSparkMax lFeeder = new CANSparkMax(100202, MotorType.kBrushless);
  private CANSparkMax rFeeder = new CANSparkMax(100102, MotorType.kBrushless);
  private CANSparkMax wrist = new CANSparkMax(100102, MotorType.kBrushless);

  private DigitalInput limitSwitch = new DigitalInput(0);

  private PIDController pidController = new PIDController(0.01, 0, 0);

  //homing

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

  public Command homeEverything() {
    return Commands.run(() -> {
      lElevator.getEncoder().setPosition(0);
      rElevator.getEncoder().setPosition(0);
      lShooter.getEncoder().setPosition(0);
      rShooter.getEncoder().setPosition(0);
      lFeeder.getEncoder().setPosition(0);
      rFeeder.getEncoder().setPosition(0);
      wrist.getEncoder().setPosition(0);
    });
  }

  public Command intake() {
    return Commands.run(() -> {
      lShooter.set(-0.1); // negative is clockwise
      rShooter.set(0.1);
      lFeeder.set(-0.1);
      rFeeder.set(0.1);
    });
  }

  public Command shoot() {
    return Commands.run(() -> {
      // spin outer wheels to 100 power
      lShooter.set(1);
      rShooter.set(-1);
    }).until(() -> {
      return lShooter.getEncoder().getVelocity() < 100;
      // TODO get this actual max value of motors
    }).andThen(Commands.run(() -> {
      lFeeder.set(1);
      rFeeder.set(-1);
    }));
  }

  public Command outtake() {
    return Commands.run(() -> {
      // spin outer wheels to 10 power
      lShooter.set(0.1);
      rShooter.set(-0.1);
      // TODO get the actual values
    }).until(() -> {
      return lShooter.getEncoder().getVelocity() < 10;
      // TODO change this value
    }).andThen(Commands.run(() -> {
      lFeeder.set(0.1);
      rFeeder.set(-0.1);
    }));
  }

  public Command moveToSpeaker() {
    //TODO put postion
    pidController.setSetpoint(120);

    return Commands.run(() -> {
      wrist.set(pidController.calculate(wrist.getEncoder().getPosition()));
    }).until(() -> {
      return wrist.getEncoder().getPosition() == 120;
    }).andThen(Commands.runOnce(() -> {
      wrist.stopMotor();
    }));
  }

  /** Creates a new ArmSubsystem. */
  public ArmSubsystem() {
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
