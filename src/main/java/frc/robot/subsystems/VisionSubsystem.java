// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

/**
 * Subsystem that reads vision data from a Limelight 4 camera using
 * RAW NETWORKTABLES — no helper library required!
 *
 * ===== TEACHING NOTE =====
 * This subsystem shows students how the Limelight works "under the hood":
 *   - The Limelight publishes data to NetworkTables under the "limelight" key.
 *   - We read individual entries (tv, tx, ty, ta, tid, botpose_wpiblue) directly.
 *   - We must know the correct data TYPES (integer vs double) for each entry.
 *
 * Compare with VisionYALLSubsystem.java which does the same thing using the
 * YALL helper library — cleaner API, but hides the NT details from you.
 * ==========================
 *
 * Key Limelight NetworkTables entries:
 *   tv  - 1 if a valid target is found, 0 otherwise  (INTEGER on Limelight 4!)
 *   tx  - horizontal offset from crosshair to target in degrees (double)
 *   ty  - vertical offset from crosshair to target in degrees (double)
 *   ta  - target area as % of image (double)
 *   tid - AprilTag ID of the primary detected tag, -1 if none (INTEGER!)
 *   botpose_wpiblue - robot pose in field space [x, y, z, roll, pitch, yaw, ...]
 */
public class VisionSubsystem extends SubsystemBase {

  private final NetworkTable limelightTable;

  // Cached values updated every periodic cycle
  private boolean hasTarget;
  private double tx;
  private double ty;
  private double ta;
  private int tagId;
  private Pose2d robotPose;

  private int debugCounter = 0;

  public VisionSubsystem() {
    limelightTable = NetworkTableInstance.getDefault().getTable("limelight");
    robotPose = new Pose2d();
  }

  @Override
  public void periodic() {
    // Read all values from NetworkTables each cycle (~20ms)
    // Use getNumber() to handle both integer and double types from Limelight
    NetworkTableEntry tvEntry = limelightTable.getEntry("tv");
    NetworkTableEntry txEntry = limelightTable.getEntry("tx");
    NetworkTableEntry tyEntry = limelightTable.getEntry("ty");
    NetworkTableEntry taEntry = limelightTable.getEntry("ta");
    NetworkTableEntry tidEntry = limelightTable.getEntry("tid");

    hasTarget = tvEntry.getInteger(0) == 1;
    tx = txEntry.getDouble(0.0);
    ty = tyEntry.getDouble(0.0);
    ta = taEntry.getDouble(0.0);
    tagId = (int) tidEntry.getInteger(-1);

    // Read the full robot pose from botpose_wpiblue (WPILib Blue Alliance origin)
    // botpose_wpiblue is [x, y, z, roll, pitch, yaw] in field-space (meters and degrees)
    // Only update the pose when we actually have a target — otherwise the array is all zeros
    double[] botpose = limelightTable.getEntry("botpose_wpiblue").getDoubleArray(new double[0]);
    if (hasTarget && botpose.length >= 6) {
      robotPose = new Pose2d(
          botpose[0], // x position in meters
          botpose[1], // y position in meters
          Rotation2d.fromDegrees(botpose[5]) // yaw rotation
      );
    }

    // Read heartbeat and latency — these change even without a target visible
    double heartbeat = limelightTable.getEntry("hb").getDouble(0.0);
    double pipelineLatency = limelightTable.getEntry("tl").getDouble(0.0);
    double captureLatency = limelightTable.getEntry("cl").getDouble(0.0);

    // Read target pose in camera space — this works WITHOUT a field map!
    // targetpose_cameraspace = [x, y, z, roll, pitch, yaw] of the tag relative to the camera
    // x = right/left, y = up/down, z = forward distance (all in meters)
    // This is great for desk testing — you'll see real distance values even
    // without uploading a field map to the Limelight.
    double[] targetPoseCam = limelightTable.getEntry("targetpose_cameraspace")
        .getDoubleArray(new double[0]);
    double camX = 0, camY = 0, camZ = 0;
    if (hasTarget && targetPoseCam.length >= 6) {
      camX = targetPoseCam[0];
      camY = targetPoseCam[1];
      camZ = targetPoseCam[2]; // forward distance to the tag
    }

    // Publish to SmartDashboard for debugging
    SmartDashboard.putBoolean("Vision/HasTarget", hasTarget);
    SmartDashboard.putNumber("Vision/TX", tx);
    SmartDashboard.putNumber("Vision/TY", ty);
    SmartDashboard.putNumber("Vision/TA", ta);
    SmartDashboard.putNumber("Vision/TagID", tagId);
    SmartDashboard.putNumber("Vision/Heartbeat", heartbeat);
    SmartDashboard.putNumber("Vision/PipelineLatencyMs", pipelineLatency);
    SmartDashboard.putNumber("Vision/CaptureLatencyMs", captureLatency);
    SmartDashboard.putNumber("Vision/RobotX", robotPose.getX());
    SmartDashboard.putNumber("Vision/RobotY", robotPose.getY());
    SmartDashboard.putNumber("Vision/RobotYaw", robotPose.getRotation().getDegrees());

    // Target-in-camera-space values (work without a field map!)
    SmartDashboard.putNumber("Vision/TargetCamX", camX);   // left/right offset (meters)
    SmartDashboard.putNumber("Vision/TargetCamY", camY);   // up/down offset (meters)
    SmartDashboard.putNumber("Vision/TargetCamZ", camZ);   // forward distance (meters)

    // Debug print every second (~50 cycles at 20ms)
    debugCounter++;
    if (debugCounter % 50 == 0) {
      // Dump the raw botpose_wpiblue array to see exactly what the Limelight sends
      StringBuilder bp = new StringBuilder("[");
      for (int i = 0; i < botpose.length; i++) {
        bp.append(String.format("%.3f", botpose[i]));
        if (i < botpose.length - 1) bp.append(", ");
      }
      bp.append("]");

      System.out.println("[Vision] tv=" + hasTarget + " tx=" + tx + " ty=" + ty  
          + " tagId=" + tagId
          + " camDist=" + String.format("%.2f", camZ) + "m"
          + " rawBotpose=" + bp.toString()
          + " pose=(" + robotPose.getX() + ", " + robotPose.getY() + ", " 
          + robotPose.getRotation().getDegrees() + ")");
    }
  }

  /** Returns true if the Limelight sees a valid AprilTag. */
  public boolean hasTarget() {
    return hasTarget;
  }

  /** Horizontal offset from crosshair to target in degrees. */
  public double getTX() {
    return tx;
  }

  /** Vertical offset from crosshair to target in degrees. */
  public double getTY() {
    return ty;
  }

  /** Target area as a percentage of the image (0-100). */
  public double getTA() {
    return ta;
  }

  /** The AprilTag ID currently being tracked, or -1 if none. */
  public int getTagId() {
    return tagId;
  }

  /**
   * Returns the robot's estimated pose on the field based on AprilTag detection.
   * Uses the Limelight's botpose which gives [x, y, z, roll, pitch, yaw].
   * The pose will be (0,0,0) if no tags are visible.
   */
  public Pose2d getRobotPose() {
    return robotPose;
  }

  /**
   * Calculate the distance from the robot's current pose to a target pose.
   * Only considers x and y (2D distance on the field floor).
   */
  public double getDistanceTo(Pose2d target) {
    return robotPose.getTranslation().getDistance(target.getTranslation());
  }

  /**
   * Calculate the angle the robot needs to turn to face a target pose.
   * Returns the angle in degrees (positive = turn left, negative = turn right).
   */
  public double getAngleTo(Pose2d target) {
    double dx = target.getX() - robotPose.getX();
    double dy = target.getY() - robotPose.getY();
    double targetAngle = Math.toDegrees(Math.atan2(dy, dx));
    double currentAngle = robotPose.getRotation().getDegrees();
    double error = targetAngle - currentAngle;

    // Normalize to [-180, 180]
    while (error > 180) error -= 360;
    while (error < -180) error += 360;

    return error;
  }
}
