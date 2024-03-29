// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkMax;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.DSControlWord;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.DriverStation;
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

  private PIDController upWristPidController = new PIDController(0.135, 0, 0.0007);
  private PIDController downWristPidController = new PIDController(0.075, 0, 0.001);

  private PIDController elevatorPidController = new PIDController(0.027, 0.000, 0.003);

  private double wristPosition = Constants.Aim.HOME_POSITION;
  private double elevatorPosition = 0;

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

    SmartDashboard.putData("PID Controller", upWristPidController);

    SmartDashboard.putNumber("Elevator Setpoint", elevatorPidController.getSetpoint());
    SmartDashboard.putNumber("Wrist Setpoint", upWristPidController.getSetpoint());

    SmartDashboard.putNumber("Wrist Speed", wrist.get());
    SmartDashboard.putBoolean("Is Intaked", isIntaked());

    SmartDashboard.putNumber("LElevator Speed", lElevator.get());
    SmartDashboard.putNumber("RElevator Speed", rElevator.get());

    // always run the PID controller
    if (DriverStation.isEnabled()) {
      upWristPidController.setSetpoint(wristPosition);
      downWristPidController.setSetpoint(wristPosition);
      if (wristPosition != Constants.Aim.HOME_POSITION) {
        wrist.set(MathUtil.clamp(upWristPidController.calculate(wrist.getEncoder().getPosition()),
            -Constants.Aim.WRIST_SPEED, Constants.Aim.WRIST_SPEED));
      } else {
        wrist.set(MathUtil.clamp(downWristPidController.calculate(wrist.getEncoder().getPosition()),
                    -Constants.Aim.WRIST_SPEED / 4, Constants.Aim.WRIST_SPEED / 4));
      }

      elevatorPidController.setSetpoint(elevatorPosition);
      lElevator.set(MathUtil.clamp(elevatorPidController.calculate(lElevator.getEncoder().getPosition()),
          -Constants.Elevator.HOME_SPEED, Constants.Elevator.HOME_SPEED));
    }
  }

  public boolean isIntaked() {
    return lFeeder.get() > 0.0 && MathUtil.isNear(0, lFeeder.getEncoder().getVelocity(), 2000);
    // && lFeeder.getEncoder().getVelocity() > -2000
  }

  public boolean shouldMoveWristJoint() {
    return MathUtil.isNear(elevatorPidController.getSetpoint(), lElevator.getEncoder().getPosition(), 6);
  }

  // stop everything
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
    }).handleInterrupt(() -> {
      stopEverythingMethod();
    });
  }

  public Command goSlowDown() {
    return Commands.run(() -> {
      wrist.set(-Constants.Aim.SLOW_PRECISION_SPEED);
    }).handleInterrupt(() -> {
      stopEverythingMethod();
    });
  }

  public Command intake() {
    return Commands.run(() -> {
      System.out.println("I AM CURRENTLY INTAKING!");
      lShooter.set(Constants.Intake.INTAKE_SPEEDS.SHOOTER_INTAKE_SPEED); // negative is clockwise
      lFeeder.set(Constants.Intake.INTAKE_SPEEDS.FEEDER_INTAKE_SPEED);
    }).handleInterrupt(this::stopEverythingMethod);
  }

  public Command shoot() {
    return Commands.runOnce(() -> {
      // spin outer wheels to 100 power
      lShooter.set(Constants.Intake.SHOOT_SPEED);
    })
        .andThen(new WaitCommand(2))
        .andThen(Commands.runOnce(() -> {
          lFeeder.set(Constants.Intake.SHOOT_SPEED);
        }))
        .andThen(new WaitCommand(2))
        .andThen(() -> {
          stopEverythingMethod();
        });
  }

  public Command dumshoot() {
    return Commands.run(() -> {
      lShooter.set(Constants.Intake.SHOOT_SPEED);
    }).handleInterrupt(() -> {
      stopEverythingMethod();
    });
  }

  public Command dumamp() {
    return Commands.run(() -> {
      lShooter.set(Constants.Intake.AMP_OUTTAKE_SPEED);
      lFeeder.set(-0.7);
    }).handleInterrupt(() -> {
      stopEverythingMethod();
    });
  }

  public Command goToAmpPosition() {
    return moveElevatorTo(Constants.Aim.ELEVATOR_HEIGHT_AMP).andThen(Commands.waitUntil(this::shouldMoveWristJoint).andThen(moveWristTo(Constants.Aim.WRIST_ANGLE_AMP)));
  }

  public Command dumbExtendElevator() {
    return Commands.run(() -> {
      elevatorPosition--;
    });
  }

  public Command dumbRetractElevator() {
    return Commands.run(() -> {
      elevatorPosition++;
    });
  }

  public Command homeEverything() {
    return moveWristTo(Constants.Aim.HOME_POSITION).andThen(moveElevatorTo(Constants.Aim.HOME_POSITION));
  }

  public Command outtake() {
    return Commands.run(() -> {
      // spin outer wheels to 10 power
      lShooter.set(-Constants.Intake.AMP_OUTTAKE_SPEED);
      lFeeder.set(-0.7);
    }).handleInterrupt(this::stopEverythingMethod);
  }
  public boolean isAtPosition() {
    return (MathUtil.isNear(elevatorPosition, lElevator.getEncoder().getPosition(), 2)) && (MathUtil.isNear(wristPosition, wrist.getEncoder().getPosition(), 2));
  }

  public Command moveElevatorTo(double pos) {
    return Commands.runOnce(() -> {
      elevatorPosition = pos;
    });
  }

  /**
   * Moves the wrist to a position.
   * 
   * @param pos the position to go to
   * @return the command to move the wrist
   */
  public Command moveWristTo(double pos) {
    return Commands.runOnce(() -> {
      wristPosition = pos;
    });
  }

  public Command scoreAmp() {
    return moveWristTo(Constants.Aim.WRIST_ANGLE_AMP).alongWith(moveElevatorTo(Constants.Aim.ELEVATOR_HEIGHT_AMP))
        .andThen(outtake()).andThen(new WaitCommand(1)).andThen(stopEverything()).andThen(homeElevator())
        .alongWith(moveWristTo(0));
  }

  public Command scoreSpeaker() {
    return moveWristTo(Constants.Aim.WRIST_ANGLE_SPEAKER)
        .alongWith(moveElevatorTo(Constants.Aim.ELEVATOR_HEIGHT_SOURCE)).andThen(shoot()).andThen(new WaitCommand(2))
        .andThen(stopEverything()).andThen(homeElevator()).alongWith(moveWristTo(0));
  }

}
