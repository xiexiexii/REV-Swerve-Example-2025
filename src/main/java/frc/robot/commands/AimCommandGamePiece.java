package frc.robot.commands;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.VisionConstants;
import frc.robot.subsystems.PhotonHelpers;
import frc.robot.subsystems.Swerve.DriveSubsystem;

// Positions Robot at the Nearest Valid Target
public class AimCommandGamePiece extends Command {
    
  // Instantiate Stuff
  DriveSubsystem m_swerveSubsystem;
  PhotonHelpers m_photonSubsystem;

  // Timer for cancellation with failed robot adjustment
  Timer timer = new Timer();

  // PID Controller stuff (woah so many so scary) 
  PIDController m_aimController = new PIDController(VisionConstants.kP_aim, VisionConstants.kI_aim, VisionConstants.kD_aim);

  /*
   * Tag Guide (Perspective is from respective DS):
   * 1: Coral Station Red Left
   * 2: Coral Station Red Right
   * 3: Processor Blue
   * 4: Barge Blue Back
   * 5: Barge Red Front
   * 6: Reef Red Front Left
   * 7: Reef Red Front Center
   * 8: Reef Red Front Right
   * 9: Reef Red Back Right
   * 10: Reef Red Back Center
   * 11: Reef Red Back Left
   * 12: Coral Station Blue Right
   * 13: Coral Station Blue Left
   * 14: Barge Blue Front
   * 15: Barge Red Back
   * 16: Processor Red
   * 17: Reef Blue Front Right
   * 18: Reef Blue Front Center
   * 19: Reef Blue Front Left
   * 20: Reef Blue Back Left
   * 21: Reef Blue Back Center
   * 22: Reef Blue Back Right
   */

  // Lil boolean for checking for "Tag In View" 
  private boolean tiv;

  // Constants
  private double m_aimTarget;

  // Constructor
  public AimCommandGamePiece(DriveSubsystem driveSubsystem, PhotonHelpers photonSubsystem) {
        
    // Definitions and setting parameters are equal to members!
    m_swerveSubsystem = driveSubsystem;
    addRequirements(driveSubsystem);
    m_photonSubsystem = photonSubsystem;
    addRequirements(photonSubsystem);
  }

  // What we do to set up the command 
  public void initialize() {

    // Reset the Shoot Commit Boolean
    VisionConstants.k_positioned = false;

    // Checks for TV
    tiv = (m_photonSubsystem.getObjectTV());

    // Set Constants
    m_aimTarget = VisionConstants.k_aimTarget;

    // Timer Reset
    timer.start();
    timer.reset();
  }
    
  // The actual control!
  public void execute() {

    VisionConstants.k_positioning = true;

    // Checks for a continued valid pose
    if (tiv){
      tiv = m_photonSubsystem.getObjectTV();
      m_swerveSubsystem.drive(0, 0, photon_aim_PID(), false);;
    }
  }

  // Add stuff we do after to reset here (a.k.a tag filters)
  public void end(boolean interrupted) {
    VisionConstants.k_positioning = false;

    if (Math.abs(m_photonSubsystem.getObjectTX() - m_aimTarget) < VisionConstants.k_aimThreshold) {
        VisionConstants.k_positioned = true;
    }
  }

  // Are we done yet? Finishes when threshold is reached or if no tag in view or if timer is reached 
  public boolean isFinished() {
    return
      // Aim (Angle)
      Math.abs(m_photonSubsystem.getObjectTX() - m_aimTarget) < VisionConstants.k_aimThreshold

      // Other quit conditions
      || !tiv || timer.get() > 3;
  }

  // Advanced PID-assisted ranging control with Photon's Yaw value from target-relative data
  private double photon_aim_PID() {

    // Camera Yaw Angle in Degrees
    m_aimController.enableContinuousInput(-30, 30);
    
    // Calculates response based on difference in angle from tag to robot
    double targetingAngularVelocity = m_aimController.calculate(m_photonSubsystem.getObjectTX() - m_aimTarget);

    // Multiply by 1 because robot is CCW Positive. Multiply by a reduction 
    // multiplier to reduce speed. Scale TX up with robot speed.
    targetingAngularVelocity *= 0.1 * DriveConstants.kMaxAngularSpeed;

    // Hooray
    return targetingAngularVelocity;
  }
}
