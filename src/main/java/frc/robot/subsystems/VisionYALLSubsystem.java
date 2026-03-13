// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import java.util.Optional;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

// --- YALL Library imports ---
// The YALL (Yet Another Limelight Library) wraps raw NetworkTables access
// into clean Java objects so you don't have to deal with NT entry types yourself.
import limelight.Limelight;
import limelight.networktables.LimelightPoseEstimator;
import limelight.networktables.LimelightPoseEstimator.EstimationMode;
import limelight.networktables.PoseEstimate;

/**
 * Subsystem that reads vision data from a Limelight 4 camera using the
 * YALL HELPER LIBRARY — a cleaner, higher-level API.
 *
 * ===== TEACHING NOTE =====
 * This subsystem shows students how to use the YALL library to read Limelight
 * data. Compare with VisionSubsystem.java (the raw NetworkTables version):
 *
 *   RAW NT (VisionSubsystem):
 *     limelightTable.getEntry("tv").getInteger(0) == 1
 *     limelightTable.getEntry("tx").getDouble(0.0)
 *     limelightTable.getEntry("botpose_wpiblue").getDoubleArray(...)
 *
 *   YALL LIBRARY (this class):
 *     limelight.getData().targetData.getTargetStatus()
 *     limelight.getData().targetData.getHorizontalOffset()
 *     poseEstimator.getPoseEstimate()
 *
 * The YALL library:
 *   ✓ Handles the correct NT data types for you (no integer vs double confusion!)
 *   ✓ Provides named methods instead of magic string keys ("tv", "tx", etc.)
 *   ✓ Wraps pose estimation into a clean PoseEstimate object
 *   ✓ Supports MegaTag1 and MegaTag2 pose estimation modes
 *
 * Tradeoff: You lose visibility into what's happening at the NT level.
 * That's why we teach BOTH approaches!
 * ==========================
 */
public class VisionYALLSubsystem extends SubsystemBase {

  // The YALL Limelight object — wraps all NT access for one camera
  private final Limelight limelight;

  // Pose estimator — lazy-initialized on first target detection.
  // The YALL LimelightPoseEstimator requires NT pose data to exist when
  // constructed, so we wait until the Limelight is actually connected and
  // publishing data before creating it.
  private LimelightPoseEstimator poseEstimator;

  // Cached values updated every periodic cycle
  private boolean hasTarget;
  private double tx;
  private double ty;
  private double ta;
  private int tagId;
  private Pose2d robotPose;

  private int debugCounter = 0;

  public VisionYALLSubsystem() {
    // Create the Limelight object — the string must match the camera's name
    // in the Limelight web UI (default is "limelight")
    limelight = new Limelight("limelight");

    // Pose estimator will be created lazily in periodic() once we have a target.
    // Creating it here (before the Limelight connects) causes a NullPointerException
    // because the internal data structures aren't populated yet.
    poseEstimator = null;

    robotPose = new Pose2d();
  }

  @Override
  public void periodic() {
    // ---- Read target data using the YALL library ----
    // Instead of limelight.getEntry("tv").getInteger(0), we just call:
    hasTarget = limelight.getData().targetData.getTargetStatus();
    tx = limelight.getData().targetData.getHorizontalOffset();
    ty = limelight.getData().targetData.getVerticalOffset();
    ta = limelight.getData().targetData.getTargetArea();
    tagId = (int) limelight.getData().targetData.getAprilTagID();

    // ---- Read pose estimate using the YALL library ----
    // Instead of manually parsing botpose_wpiblue double arrays, we get a
    // nice PoseEstimate object with pose, timestamp, tag count, etc.
    //
    // IMPORTANT: We lazy-initialize the pose estimator on first target detection.
    // The YALL library's internal data structures (HashMap) aren't populated until
    // the Limelight connects and publishes pose data over NetworkTables.
    // Creating the estimator too early causes NullPointerException.
    if (hasTarget) {
      try {
        // Lazy init: create pose estimator once we know the Limelight is connected
        if (poseEstimator == null) {
          poseEstimator = limelight.createPoseEstimator(EstimationMode.MEGATAG1);
          System.out.println("[VisionYALL] Pose estimator initialized (MegaTag1)");
        }
        Optional<PoseEstimate> estimate = poseEstimator.getPoseEstimate();
        if (debugCounter % 50 == 0) {
          System.out.println("[VisionYALL POSE DEBUG]"
              + " isPresent=" + estimate.isPresent()
              + (estimate.isPresent()
                  ? " hasData=" + estimate.get().hasData
                    + " tagCount=" + estimate.get().tagCount
                    + " pose3d=(" + estimate.get().pose.getX()
                    + ", " + estimate.get().pose.getY()
                    + ", " + estimate.get().pose.getZ() + ")"
                    + " timestamp=" + estimate.get().timestampSeconds
                  : ""));
        }
        if (estimate.isPresent() && estimate.get().hasData) {
          PoseEstimate pe = estimate.get();
          robotPose = pe.pose.toPose2d();
        }
      } catch (NullPointerException e) {
        // If it still fails, reset and try again next cycle
        poseEstimator = null;
        if (debugCounter % 50 == 0) {
          System.out.println("[VisionYALL POSE DEBUG] NullPointerException — will retry: " + e.getMessage());
        }
      }
    }

    // ---- Read target pose in camera space using the YALL library ----
    // getCameraToTarget() returns a Pose3d of the tag relative to the camera.
    // This works WITHOUT a field map — great for desk testing!
    //
    // Compare with the raw NT version which reads "targetpose_cameraspace" manually.
    // Here, YALL gives us a clean Pose3d object.
    double camX = 0, camY = 0, camZ = 0;
    if (hasTarget) {
      Pose3d camToTarget = limelight.getData().targetData.getCameraToTarget();
      camX = camToTarget.getX();   // left/right offset (meters)
      camY = camToTarget.getY();   // up/down offset (meters) 
      camZ = camToTarget.getZ();   // forward distance (meters)
    }

    // Publish to SmartDashboard under "VisionYALL/" prefix
    // (so you can compare side-by-side with the raw NT version's "Vision/" prefix)
    SmartDashboard.putBoolean("VisionYALL/HasTarget", hasTarget);
    SmartDashboard.putNumber("VisionYALL/TX", tx);
    SmartDashboard.putNumber("VisionYALL/TY", ty);
    SmartDashboard.putNumber("VisionYALL/TA", ta);
    SmartDashboard.putNumber("VisionYALL/TagID", tagId);
    SmartDashboard.putNumber("VisionYALL/RobotX", robotPose.getX());
    SmartDashboard.putNumber("VisionYALL/RobotY", robotPose.getY());
    SmartDashboard.putNumber("VisionYALL/RobotYaw", robotPose.getRotation().getDegrees());

    // Target-in-camera-space values (work without a field map!)
    SmartDashboard.putNumber("VisionYALL/TargetCamX", camX);
    SmartDashboard.putNumber("VisionYALL/TargetCamY", camY);
    SmartDashboard.putNumber("VisionYALL/TargetCamZ", camZ);

    // Debug print every second (~50 cycles at 20ms)
    debugCounter++;
    if (debugCounter % 50 == 0) {
      System.out.println("[VisionYALL] tv=" + hasTarget + " tx=" + tx + " ty=" + ty
          + " tagId=" + tagId
          + " camDist=" + String.format("%.2f", camZ) + "m"
          + " pose=(" + robotPose.getX() + ", " + robotPose.getY() + ", "
          + robotPose.getRotation().getDegrees() + ")");
    }
  }

  // ======================================================================
  // PUBLIC API — identical to VisionSubsystem so commands can use either!
  // ======================================================================

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
   * Uses MegaTag1 pose estimation from the YALL library.
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
