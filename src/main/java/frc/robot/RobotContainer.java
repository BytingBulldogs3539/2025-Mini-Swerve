// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;
import com.pathplanner.lib.commands.PathPlannerAuto;

import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;

//import frc.robot.constants.*;


/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and trigger mappings) should be declared here.
 */
public class RobotContainer {
  public static DriveSubsystem driveSubsystem = TunerConstants.createDrivetrain(); // TunerConstants.DriveTrain

  public static ServoSubsystem servoSubsystem = new ServoSubsystem();
  public static IntakeSubsystem intakeSubsystem = new IntakeSubsystem();

  // The robot's subsystems and commands are defined here...

  // Replace with CommandPS4Controller or CommandJoystick if needed

  // public static DriveSubsystem DriveSubsystem = new DriveSubsystem();

  
	public static CommandXboxController driverController = new CommandXboxController(1);
	public static CommandXboxController operatorController = new CommandXboxController(0);

  
	public static Trigger rightDriverTrigger = driverController.rightTrigger(0.5);
	public static Trigger rightDriverBumper = driverController.rightBumper();

  
	public static SendableChooser<Command> chooser = new SendableChooser<Command>();


  public RobotContainer() {
    NamedCommands.registerCommand("OpenServoCommand", new ServoCommand(Constants.clawOpenPosition));
    NamedCommands.registerCommand("ClosedServoCommand", new ServoCommand(Constants.clawClosedPosition));

    // Configure the trigger bindings
    configureBindings();
    putAutons();
  }
  public void putAutons()
  {
    chooser = AutoBuilder.buildAutoChooser();
		SmartDashboard.putData(chooser);
  }

  /**
   * Use this method to define your trigger->command mappings. Triggers can be created via the
   * {@link Trigger#Trigger(java.util.function.BooleanSupplier)} constructor with an arbitrary
   * predicate, or via the named factories in {@link
   * edu.wpi.first.wpilibj2.command.button.CommandGenericHID}'s subclasses for {@link
   * CommandXboxController Xbox}/{@link edu.wpi.first.wpilibj2.command.button.CommandPS4Controller
   * PS4} controllers or {@link edu.wpi.first.wpilibj2.command.button.CommandJoystick Flight
   * joysticks}.
   */
  private void configureBindings() {

    driveSubsystem.setDefaultCommand(new DriveCommand());
    driverController.rightBumper().whileTrue(new ServoCommand(Constants.clawClosedPosition));
    driverController.leftBumper().whileTrue(new ServoCommand(Constants.clawOpenPosition));
    // operatorController.rightBumper().whileTrue(new ReverseIntakeCommand());

  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    // This method loads the auto when it is called, however, it is recommended
    // to first load your paths/autos when code starts, then return the
    // pre-loaded auto/path
    return chooser.getSelected();
  }
}
