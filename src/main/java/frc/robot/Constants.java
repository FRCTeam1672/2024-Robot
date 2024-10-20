// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.pathplanner.lib.util.PIDConstants;

import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.util.Units;
import swervelib.math.Matter;
import swervelib.parser.PIDFConfig;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean constants. This
 * class should not be used for any other purpose. All constants should be declared globally (i.e. public static). Do
 * not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants
{

  public static final double ROBOT_MASS = (148 - 20.3) * 0.453592; // 32lbs * kg per pound
  public static final Matter CHASSIS    = new Matter(new Translation3d(0, 0, Units.inchesToMeters(8)), ROBOT_MASS);
  public static final double LOOP_TIME  = 0.13; //s, 20ms + 110ms sprk max velocity lag

  public static final class Auton
  {

    public static final PIDFConfig TranslationPID = new PIDFConfig(2.5, 0, 0);
    public static final PIDFConfig angleAutoPID   = new PIDFConfig(0.8, 0, 0);

    public static final double MAX_ACCELERATION = 3;
  }

  public static final class Drivebase
  {

    // Hold time on motor brakes when disabled
    public static final double WHEEL_LOCK_TIME = 10; // seconds
  }

  public static final class Intake {
    // its not about speed its about power (speed = power)
    public static final class INTAKE_SPEEDS {
      public static final double FEEDER_INTAKE_SPEED = 0.4;
      public static final double SHOOTER_INTAKE_SPEED = 0.1;
    }
    public static final double AMP_OUTTAKE_SPEED = 0.3;
    public static final double SHOOT_SPEED = -1;
    public static final double OUTTAKE_VELOCITY = 1200;
    public static final int SHOOT_VELOCITY = 6000;
  }
  /**
   * This class is for aiming. 
   * @param nothing its legit nothing dumbass
   */
  public static final class Aim {
    public static final double HOME_POSITION = -2; 
    public static final double WRIST_ANGLE_AMP = -17;
    public static final double WRIST_ANGLE_SPEAKER = HOME_POSITION; //asgfawieasfafeawefawefaergjuwGAWEFLIHAEGIUAHERGIAEHGAEIGUHAFJBASDJBADJFASDJLGHASDIFLAEIULAIWULHGIAUEHFKASDHFGJKADHG
    public static final double ELEVATOR_HEIGHT_AMP = -80.5;
    //THIS IS CHANGED
    //BEFORE

    //AFTER
    public static final double ELEVATOR_HEIGHT_SOURCE = -71;//aksefhawefhGPUIeqehrghquierghuieqrhghadfjkghkadghuishguhsekgjhsdjkfghadughauighlafklahawilawlegaagalalkgaelg
    public static final double WRIST_SPEED = 0.6;
    public static final double SLOW_PRECISION_SPEED = -0.1;
  }
  public static final class AutonConstants
  {

    public static final PIDConstants TRANSLATION_PID = new PIDConstants(4, 0, 0);
    public static final PIDConstants ANGLE_PID  = new PIDConstants(0.9, 0, 0.02);
  }
  public static final class Elevator {
    public static final double HOME_SPEED = 0.3;
  }

  public static final class OperatorConstants
  {

    // Joystick Deadband
    public static final double LEFT_X_DEADBAND  = 0.1;
    public static final double LEFT_Y_DEADBAND  = 0.1;
    public static final double RIGHT_X_DEADBAND = 0.1;
    public static final double TURN_CONSTANT    = 6;
  }
}
