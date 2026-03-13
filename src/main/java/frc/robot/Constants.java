// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide
 * numerical or boolean constants. This class should not be used for any other
 * purpose. All constants should be declared globally (i.e. public static). Do
 * not put anything functional in this class.
 *
 * <p>
 * It is advised to statically import this class (or one of its inner classes)
 * wherever the constants are needed, to reduce verbosity.
 */
public final class Constants {
  public static final class DriveConstants {
    // Motor controller IDs for drivetrain motors
    public static final int LEFT_LEADER_ID = 1;
    public static final int LEFT_FOLLOWER_ID = 2;
    public static final int RIGHT_LEADER_ID = 3;
    public static final int RIGHT_FOLLOWER_ID = 4;

    // Current limit for drivetrain motors. 60A is a reasonable maximum to reduce
    // likelihood of tripping breakers or damaging CIM motors
    public static final int DRIVE_MOTOR_CURRENT_LIMIT = 60;
  }

  public static final class FuelConstants {
    // Motor controller IDs for Fuel Mechanism motors
    public static final int FEEDER_MOTOR_ID = 6;
    public static final int INTAKE_LAUNCHER_MOTOR_ID = 5;

    // Current limit and nominal voltage for fuel mechanism motors.
    public static final int FEEDER_MOTOR_CURRENT_LIMIT = 60;
    public static final int LAUNCHER_MOTOR_CURRENT_LIMIT = 60;

    // Voltage values for various fuel operations. These values may need to be tuned
    // based on exact robot construction.
    // See the Software Guide for tuning information
    public static final double INTAKING_FEEDER_VOLTAGE = -12;
    public static final double INTAKING_INTAKE_VOLTAGE = 10;
    public static final double LAUNCHING_FEEDER_VOLTAGE = 9;
    public static final double LAUNCHING_LAUNCHER_VOLTAGE = 10.6;
    public static final double SPIN_UP_FEEDER_VOLTAGE = -6;
    public static final double SPIN_UP_SECONDS = 1;
  }

  public static final class OperatorConstants {
    // Port constants for driver and operator controllers. These should match the
    // values in the Joystick tab of the Driver Station software
    public static final int DRIVER_CONTROLLER_PORT = 0;
    public static final int OPERATOR_CONTROLLER_PORT = 1;

    // This value is multiplied by the joystick value when rotating the robot to
    // help avoid turning too fast and beign difficult to control
    public static final double DRIVE_SCALING = .7;
    public static final double ROTATION_SCALING = .8;
  }

  public static final class VisionConstants {
    // Proportional gain for turning toward the target.
    // Higher = turns faster, but too high will overshoot and oscillate.
    // Start low and increase until the robot turns smoothly to the target.
    public static final double TURN_P_GAIN = 0.02;

    // Proportional gain for driving toward the target.
    // Higher = drives faster, but too high will overshoot the position.
    public static final double DRIVE_P_GAIN = 0.5;

    // Maximum speeds (0 to 1) to prevent the robot from going too fast
    // during vision-assisted driving
    public static final double MAX_TURN_SPEED = 0.4;
    public static final double MAX_DRIVE_SPEED = 0.5;

    // How close is "close enough" to the target position
    public static final double POSITION_TOLERANCE_METERS = 0.1; // ~4 inches
    public static final double ANGLE_TOLERANCE_DEGREES = 5.0;   // within 5 degrees
  }

  /**
   * Example field positions for the 2026 game.
   * These are placeholder coordinates — you'll update them with real field
   * measurements once you know your game and field layout.
   *
   * The FRC field is approximately 16.54m x 8.02m.
   * Origin (0,0) is at the bottom-left corner when viewed from the blue alliance
   * driver station.
   */
  public static final class FieldPositions {
    // Example: a scoring position in front of a game element
    public static final Pose2d SCORE_POSITION = new Pose2d(2.0, 5.5, Rotation2d.fromDegrees(0));

    // Example: a pickup/intake position
    public static final Pose2d PICKUP_POSITION = new Pose2d(14.0, 4.0, Rotation2d.fromDegrees(180));

    // Example: a centered starting position
    public static final Pose2d START_POSITION = new Pose2d(1.5, 4.0, Rotation2d.fromDegrees(0));
  }
}
