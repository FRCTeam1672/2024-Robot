// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.arm;

import com.revrobotics.CANSparkBase;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.math.MathUsageId;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class ElevatorSubsystem extends SubsystemBase {
  /** Creates a new ElevatorSubsystem. */
  private CANSparkMax left = new CANSparkMax(Constants.CAN_ID.LEFT_ELEVATOR, MotorType.kBrushless);
  private CANSparkMax right = new CANSparkMax(Constants.CAN_ID.RIGHT_ELEVATOR, MotorType.kBrushless);

  private PIDController elevatorController = new PIDController(0.1, 0, 0);

  public ElevatorSubsystem() {
    left.follow(right);
    left.setIdleMode(IdleMode.kBrake);
    right.setIdleMode(IdleMode.kBrake);
  }

  @Override
  public void periodic() {

  }

  public Command extendToPosition(int position) {
    elevatorController.setSetpoint(position);
    return Commands.run(() -> {
      left.set(
        MathUtil.clamp(
          -elevatorController.calculate(left.getEncoder().getPosition()), 
          position, 
          position
        )
      );
    }, this).until(
      () -> {
        return MathUtil.isNear(position, left.getEncoder().getPosition(), 10);
      }
    ).withTimeout(3);
  }
}
