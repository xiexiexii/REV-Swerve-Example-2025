package frc.robot.subsystems;

import static edu.wpi.first.units.Units.Meter;

import java.util.List;

import org.photonvision.PhotonCamera;
import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

// Here we handle a whole bunch of random stuff so you don't
// have to do it manually. Hooray!
public class PhotonHelpers extends SubsystemBase {

  // New Cameras
  private PhotonCamera m_aprilTagCamera;
  private PhotonCamera m_objectDetectionCamera;
  
  // Results
  private List<PhotonPipelineResult> m_aprilTagResults;
  private List<PhotonPipelineResult> m_objectDetectionResults;

  // Latest and coolest data
  private PhotonPipelineResult m_latestAprilTagResult;
  private PhotonPipelineResult m_latestObjectDetectionResult;

  // 3D Apriltag Tracking
  private Transform3d m_pose3D;
  
  // You know the drill...
  public PhotonHelpers(String cameraName, Boolean isAprilTags) {
    
    // Apriltags
    if (isAprilTags) m_aprilTagCamera = new PhotonCamera(cameraName);

    // Object
    else m_objectDetectionCamera = new PhotonCamera(cameraName);
    
    // 3D Pose for Hooray
    m_pose3D = new Transform3d();
  }

  // Different drill, same drill (iykyk)...
  public PhotonHelpers(String apriltagCameraName, String objectCameraName) {
    
    // Apriltags 
    m_aprilTagCamera = new PhotonCamera(apriltagCameraName);

    // Object 
    m_objectDetectionCamera = new PhotonCamera(objectCameraName);
    
    // 3D Pose for Hooray
    m_pose3D = new Transform3d();
  }

  // Stuff that should just keep on happening ig
  public void periodic() {

    // Apriltag
    if (m_aprilTagCamera != null) {
      m_aprilTagResults = m_aprilTagCamera.getAllUnreadResults();
      if (!m_aprilTagResults.isEmpty()) {
        m_latestAprilTagResult = m_aprilTagResults.get(m_aprilTagResults.size() - 1);
        m_pose3D = getPose3D();
      }

      // Apriltag SmartDashboard shenanigans (Uncomment what you need)
      // SmartDashboard.putNumber("Photonvision/Apriltags/Tag Pitch", getPitch3D());
      // SmartDashboard.putNumber("Photonvision/Apriltags/Tag Roll", getRoll3D());
      SmartDashboard.putNumber("Photonvision/Apriltags/Tag Yaw", getYaw3D());
    }

    // Object Detection
    if (m_objectDetectionCamera != null) {
      m_objectDetectionResults = m_objectDetectionCamera.getAllUnreadResults();
      if (!m_objectDetectionResults.isEmpty()) {
        m_latestObjectDetectionResult = m_objectDetectionResults.get(m_objectDetectionResults.size() - 1);
      }

      // Same for object stuff (Uncomment what you need)
      SmartDashboard.putNumber("Photonvision/Object/Object TX", getObjectTX());
      SmartDashboard.putNumber("Photonvision/Object/Object TY", getObjectTY());
      SmartDashboard.putBoolean("PhotonVision/Object/Object TV", getObjectTV());
    }
  }

  /*
  ********************************
  **  METHODS FOR 2D APRILTAGS  **
  ********************************
  */

  // Get Apriltag Target data
  public List<PhotonPipelineResult> getAprilTagResults() {
    return m_aprilTagResults;
  }

  // Get Latest Result
  public PhotonPipelineResult getLatestAprilTagResult() {
    return m_latestAprilTagResult;
  }

  // Get TV Status
  public boolean getTV() {
    return m_latestAprilTagResult.hasTargets();
  }

  // Get TX
  public double getTX() {
    if (m_latestAprilTagResult.hasTargets()) {
      return m_latestAprilTagResult.getBestTarget().getYaw();
    }
    else return 0.0;
  }

  // Get TY
  public double getTY() {
    if (m_latestAprilTagResult.hasTargets()) {
      return m_latestAprilTagResult.getBestTarget().getPitch();
    }
    else return 0.0;
  }

  // Get TA
  public double getTA() {
    if (m_latestAprilTagResult.hasTargets()) {
      return m_latestAprilTagResult.getBestTarget().getArea();
    }
    else return 0.0;
  }

  // Get Fiducial ID
  public int getFiducialID() {
    if (m_latestAprilTagResult.hasTargets()) {
      return m_latestAprilTagResult.getBestTarget().getFiducialId();
    }
    else return -1;
  }

  /*
   ********************************
   **  METHODS FOR 3D APRILTAGS  **
   **    NOTE: CAMERA MUST BE    **
   **    CALIBRATED TO USE!!!    **
   ********************************
  */

  // Get Pose3D for 3D Tag Tracking - CAMERA MUST BE CALIBRATED!
  public Transform3d getPose3D() {
    if (m_latestAprilTagResult.hasTargets()) {
      return m_latestAprilTagResult.getBestTarget().getBestCameraToTarget();
    }
    else return new Transform3d();
  }

  // Get TX3D (Forwards/Backwards, 3D Tracking)
  public double getBotPoseTargetSpaceTX3D() {
    if (m_latestAprilTagResult.hasTargets()) {
      return m_pose3D.getMeasureX().in(Meter);
    }
    else return 0.0;
  }

  // Get TY3D (Left/Right, 3D Tracking)
  public double getBotPoseTargetSpaceTY3D() {
    if (m_latestAprilTagResult.hasTargets()) {
      return m_pose3D.getMeasureY().in(Meter);
    }
    else return 0.0;
  }

  // Get TZ3D (Up/Down, 3D Tracking)
  public double getBotPoseTargetSpaceTZ3D() {
    if (m_latestAprilTagResult.hasTargets()) {
      return m_pose3D.getMeasureZ().in(Meter);
    }
    else return 0.0;
  }

  // Get Roll (Angle X, 3D Tracking)
  public double getRoll3D() {
    if (m_latestAprilTagResult.hasTargets()) {
      return MathUtil.angleModulus(m_pose3D.getRotation().getX() - Math.PI) * (180 / Math.PI);
    }
    else return 0.0;
  }

  // Get Pitch (Angle Y, 3D Tracking)
  public double getPitch3D() {
    if (m_latestAprilTagResult.hasTargets()) {
      return MathUtil.angleModulus(m_pose3D.getRotation().getY() - Math.PI) * (180 / Math.PI);
    }
    else return 0.0;
  }

  // Get Yaw (Angle Z, 3D Tracking)
  public double getYaw3D() {
    if (m_latestAprilTagResult.hasTargets()) {
      return MathUtil.angleModulus(m_pose3D.getRotation().getZ() - Math.PI) * (180 / Math.PI);
    }
    else return 0.0;
  }

  // Search for Specific Tag
  public boolean hasTag(int tagID) {
    for (PhotonTrackedTarget tag : m_latestAprilTagResult.getTargets()) {
      if (tag.getFiducialId() == tagID) {
        return true;
      }
    }
    return false;
  }

  /*
  ************************************
  **   METHODS FOR OBJ. DETECTION   **
  ************************************
  */

  // Get Object Detection data
  public List<PhotonPipelineResult> getObjectDetectionResults() {
    return m_objectDetectionResults;
  }

  // Get Latest Result
  public PhotonPipelineResult getLatestObjectDetectionResult() {
    return m_latestObjectDetectionResult;
  }

  // Get Object TV Status
  public boolean getObjectTV() {
    return m_latestObjectDetectionResult.hasTargets();
  }

  // Get Object TX
  public double getObjectTX() {
    if (m_latestObjectDetectionResult.hasTargets()) {
      return m_latestObjectDetectionResult.getBestTarget().getYaw();
    }
    else return 0.0;
  }

  // Get Object TY
  public double getObjectTY() {
    if (m_latestObjectDetectionResult.hasTargets()) {
      return m_latestObjectDetectionResult.getBestTarget().getPitch();
  }
  else return 0.0;
  }

  // Get Object TA
  public double getObjectTA() {
    if (m_latestObjectDetectionResult.hasTargets()) {
      return m_latestObjectDetectionResult.getBestTarget().getArea();
  }
  else return 0.0;
  }

  // Get Object Class ID
  public int getObjectClassID() {
    if (m_latestObjectDetectionResult.hasTargets()) {
      return m_latestObjectDetectionResult.getBestTarget().getDetectedObjectClassID();
  }
  else return -1;
  }

  // Get Object Confidence
  public double getObjectConfidence() {
    if (m_latestObjectDetectionResult.hasTargets()) {
      return m_latestObjectDetectionResult.getBestTarget().getDetectedObjectConfidence();
  }
  else return -1;
  }
}