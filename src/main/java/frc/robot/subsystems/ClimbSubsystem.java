// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkMax;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class ClimbSubsystem extends SubsystemBase {
  private CANSparkMax lClimb = new CANSparkMax(21, MotorType.kBrushless);
  private CANSparkMax rClimb = new CANSparkMax(22, MotorType.kBrushless);

  private PIDController climbPidController = new PIDController(0, 0, 0);

  private double climbPosition = 0;
  /** Creates a new ClimbSubsystem. */
  public ClimbSubsystem() {
    rClimb.follow(lClimb, true);
  }

  @Override
  public void periodic() {
    SmartDashboard.putNumber("LClimb position", lClimb.getEncoder().getPosition());
    SmartDashboard.putNumber("RClimb position", rClimb.getEncoder().getPosition());
    // This method will be called once per scheduler run
    if(DriverStation.isEnabled()) {
      // climbPidController.setSetpoint(climbPosition);
      // lClimb.set(MathUtil.clamp(climbPidController.calculate(lClimb.getEncoder().getPosition()), -.3, .3));
    }
  }

  public void stop() {
      lClimb.stopMotor();
      rClimb.stopMotor();
  }
  public Command stopCommand() {
    return Commands.runOnce(() -> {
      stop();
    });
  }

  public Command goUp() {
    return Commands.run(() -> {
      lClimb.set(-.75);
    });
  }

  public Command smartUp() {
    return Commands.runOnce(() -> {
      climbPosition = 1000000000.0;
    });
  }

  public Command goDown() {
    return Commands.run(() -> {
      lClimb.set(.75);
    });
  }

  public Command smartDown() {
    return Commands.runOnce(() -> {
      climbPosition = 0.0;
    });
  }
}
