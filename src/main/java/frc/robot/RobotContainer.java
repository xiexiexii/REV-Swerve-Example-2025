// Imports stuff (again!)

package frc.robot;

import com.pathplanner.lib.auto.NamedCommands;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants.ControllerConstants;
import frc.robot.commands.LEDColorChangeCommand;
import frc.robot.subsystems.LEDSubsystem;
import frc.robot.subsystems.Swerve.DriveSubsystem;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;

// This class is where the bulk of the robot should be declared.  Since Command-based is a
// "declarative" paradigm, very little robot logic should actually be handled in the Robot
// periodic methods (other than the scheduler calls).  Instead, the structure of the robot
// (including subsystems, commands, and button mappings) should be declared here.

public class RobotContainer {

  // Robot's Subsystems
  private final DriveSubsystem m_robotDrive = new DriveSubsystem();
  private final LEDSubsystem m_LEDSubsystem = new LEDSubsystem();
  SendableChooser<Command> m_chooser = new SendableChooser<>();

  // Controllers
  XboxController m_driverController = new XboxController(ControllerConstants.kDriverControllerPort);

  // The container for the robot. Contains subsystems, OI devices, and commands.
  public RobotContainer() {

    // Configure the button bindings
    configureButtonBindings();

    // Configure Default Commands
    m_robotDrive.setDefaultCommand(
      
      // Translation of the robot is controlled by the left stick
      // Turning is controlled by the X axis of the right stick
      new RunCommand(
        () -> m_robotDrive.drive(
          -MathUtil.applyDeadband(m_driverController.getLeftY(), ControllerConstants.kDriveDeadband),
          -MathUtil.applyDeadband(m_driverController.getLeftX(), ControllerConstants.kDriveDeadband),
          -MathUtil.applyDeadband(m_driverController.getRightX(), ControllerConstants.kDriveDeadband),
          true),
        m_robotDrive
      )
    );

    // Chooser window on SmartDashboard/Shuffleboard/Elastic
    SmartDashboard.putData("AutoMode", m_chooser);

    // Named Command Configuration
    NamedCommands.registerCommand("Change LED Color", new LEDColorChangeCommand(m_LEDSubsystem));

    // Autos
    m_chooser.addOption("Curvy yay", m_robotDrive.getAuto("Curvy yay"));
  }

  // Define Button and Axis bindings here
  private void configureButtonBindings() {

    // Sets wheels in an X position to prevent movement - A
    new JoystickButton(m_driverController, ControllerConstants.k_A)
      .whileTrue(new RunCommand(() -> m_robotDrive.setX(), m_robotDrive)
    );
  }

  public Command getAutonomousCommand() {
    return m_chooser.getSelected();   
  } 
}