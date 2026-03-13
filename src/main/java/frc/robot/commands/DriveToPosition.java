// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.CANDriveSubsystem;
import frc.robot.subsystems.VisionSubsystem;
import static frc.robot.Constants.VisionConstants.*;

/**
 * Drives the robot to a specific position on the field using Limelight AprilTag
 * pose estimation.
 *
 * How it works:
 * 1. Each cycle, the VisionSubsystem provides the robot's current field pose
 *    (from Limelight AprilTag detection)
 * 2. We calculate the angle to the target and the distance remaining
 * 3. We use a simple proportional controller to:
 *    - Turn toward the target (rotation)
 *    - Drive toward the target (forward speed)
 * 4. The command finishes when we're within tolerance of the target position
 *
 * This is a basic "drive to pose" — more advanced versions use PID controllers,
 * motion profiling, or path planning libraries like PathPlanner.
 */
public class DriveToPosition extends Command {

  private final CANDriveSubsystem driveSubsystem;
  private final VisionSubsystem visionSubsystem;
  private final Pose2d targetPose;

  /**
   * Creates a new DriveToPosition command.
   *
   * @param driveSubsystem  the drive subsystem to control motors
   * @param visionSubsystem the vision subsystem to get robot pose
   * @param targetPose      the target pose to drive to (field coordinates)
   */
  public DriveToPosition(CANDriveSubsystem driveSubsystem, VisionSubsystem visionSubsystem,
      Pose2d targetPose) {
    this.driveSubsystem = driveSubsystem;
    this.visionSubsystem = visionSubsystem;
    this.targetPose = targetPose;

    // This command needs BOTH subsystems — it reads vision and drives motors
    addRequirements(driveSubsystem);
    // Note: we don't addRequirements(visionSubsystem) because we only read from
    // it. Multiple commands should be able to read vision data simultaneously.
  }

  @Override
  public void initialize() {
    // Nothing to set up — we calculate everything in execute()
  }

  @Override
  public void execute() {
    // If we can't see any AprilTags, stop and wait for a sighting
    if (!visionSubsystem.hasTarget()) {
      driveSubsystem.driveArcade(0, 0);
      return;
    }

    // Calculate how far we need to turn and how far we need to drive
    double angleError = visionSubsystem.getAngleTo(targetPose);
    double distance = visionSubsystem.getDistanceTo(targetPose);

    // --- Proportional control for rotation ---
    // Multiply the angle error by a gain to get a rotation speed.
    // Clamp it so we don't spin too fast.
    double rotation = angleError * TURN_P_GAIN;
    rotation = clamp(rotation, -MAX_TURN_SPEED, MAX_TURN_SPEED);

    // --- Proportional control for forward speed ---
    // Only drive forward if we're roughly pointing at the target (within 30°).
    // This prevents the robot from driving sideways while still turning.
    double speed = 0;
    if (Math.abs(angleError) < 30) {
      speed = distance * DRIVE_P_GAIN;
      speed = clamp(speed, -MAX_DRIVE_SPEED, MAX_DRIVE_SPEED);
    }

    driveSubsystem.driveArcade(speed, rotation);
  }

  @Override
  public void end(boolean interrupted) {
    // Always stop the motors when the command ends
    driveSubsystem.driveArcade(0, 0);
  }

  @Override
  public boolean isFinished() {
    // We're "there" when we're close enough in both position and angle
    double distance = visionSubsystem.getDistanceTo(targetPose);
    double angleError = Math.abs(visionSubsystem.getAngleTo(targetPose));

    return distance < POSITION_TOLERANCE_METERS && angleError < ANGLE_TOLERANCE_DEGREES;
  }

  /** Clamp a value between a minimum and maximum. */
  private double clamp(double value, double min, double max) {
    return Math.max(min, Math.min(max, value));
  }
}
