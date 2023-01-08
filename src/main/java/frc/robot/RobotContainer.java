// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.io.File;
import java.io.IOException;
import java.nio.file.Path;
import java.util.ArrayList;
import java.util.HashMap;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryUtil;
import edu.wpi.first.util.sendable.Sendable;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import frc.robot.Constants.AutoConstants;
import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.OIConstants;
import frc.robot.commands.ExampleCommand;
import frc.robot.commands.SwerveWithJoystick;
import frc.robot.subsystems.ExampleSubsystem;
import frc.robot.subsystems.SwerveSubsystem;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.PrintCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.SwerveControllerCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;

import com.pathplanner.lib.PathConstraints;
import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.PathPlannerTrajectory;
import com.pathplanner.lib.PathPoint;
import com.pathplanner.lib.auto.PIDConstants;
import com.pathplanner.lib.auto.SwerveAutoBuilder;
import com.pathplanner.lib.commands.FollowPathWithEvents;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
  // The robot's subsystems and commands are defined here...
   private final ExampleSubsystem m_exampleSubsystem = new ExampleSubsystem();
   private final ExampleCommand m_autoCommand = new ExampleCommand(m_exampleSubsystem);

    private final SwerveSubsystem swerveSubsystem = new SwerveSubsystem();

    private final XboxController driverJoystick = new XboxController(OIConstants.kDriverControllerPort);

    SendableChooser<Command> autoChooser = new SendableChooser<>();



  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() 
  {

    // the exclamation mark is to ensure that the default is that the robot will operate in the field reference frame
    swerveSubsystem.setDefaultCommand(new SwerveWithJoystick(
      swerveSubsystem,
      () -> -driverJoystick.getRawAxis(OIConstants.kDriverYAxis), 
      () -> driverJoystick.getRawAxis(OIConstants.kDriverXAxis), 
      () -> driverJoystick.getRawAxis(OIConstants.kDriverRotAxis), 
      () -> !driverJoystick.getRawButton(OIConstants.kDriverFieldOrientedButtonIdx)));
    // Configure the button bindings
    configureButtonBindings();





    //maybe reformat this to do what brandon moe did? instead create an array with each of the files here that stores all of the paths.
/* 
    autoChooser.addOption("Curvy path", loadPathPlannerTrajectory("src/main/deploy/pathplanner/generatedJSON/Curvy Path.wpilib.json", true));
    autoChooser.addOption("Straight Path", loadPathPlannerTrajectory("src/main/deploy/pathplanner/generatedJSON/Straight Path.wpilib.json", true));
*/





    try 
    {
      // Create a file object
      File f = new File("./src/main/deploy/pathplanner");

      // Get all the names of the files present
      // in the given directory
      File[] files = f.listFiles();
      System.out.println("Files are:");
      // Display the names of the files
      for (int i = 0; i < files.length; i++) 
      {
        String file_name = files[i].getName();
        String file_extention = file_name.substring(file_name.length() - 5, file_name.length());
        String path_name = file_name.substring(0, file_name.length() - 5);
        String auto_name = file_name;



        /*
        if (file_extention.equals(".path"))
        {

          //might have to change the file name to auto name and just throw in a substring method to get the actual name of the path.
          autoChooser.addOption(file_name, loadPathPlannerTrajectory(path_name, true));

        }
        */
      }
    }
    catch (Exception e) 
    {
      System.err.println(e.getMessage());
    }
    Shuffleboard.getTab("Autonomous").add(autoChooser);
  }


  //Original method to use swerve auto. This may work, the reason I wrote up the other piece of code is because 
  //it uses a pathPlanerTrajectory rather than a WPILIB trajecotry. Path Planner and its wiki states that they convert
  //their version 
  /*
  public Command loadPathPlannerTrajectory(String filename, boolean resetOdometry)
  {
    Trajectory trajectory;
    try 
    {
      Path trajectoryPath = Filesystem.getDeployDirectory().toPath().resolve(filename); 
      trajectory = TrajectoryUtil.fromPathweaverJson(trajectoryPath);
    } catch (IOException exception) 
    {
        DriverStation.reportError("Unable to open trajectory", exception.getStackTrace());
        System.out.println("Unable to read from file");
        return new InstantCommand();
    }

    PIDController xController = new PIDController(AutoConstants.kPXController, 0, 0);
    PIDController yController = new PIDController(AutoConstants.kPYController, 0, 0);
    ProfiledPIDController thetaController = new ProfiledPIDController(AutoConstants.kPThetaController, 0, 0, AutoConstants.kThetaControllerConstraints);
    thetaController.enableContinuousInput(-Math.PI, Math.PI);

    SwerveControllerCommand swerveControllerCommand = new SwerveControllerCommand(
      trajectory, 
      swerveSubsystem::getPose, 
      DriveConstants.kDriveKinematics, 
      xController, 
      yController, 
      thetaController, 
      swerveSubsystem::setModuleStates, 
      swerveSubsystem);


    if(resetOdometry)
    {
      return new SequentialCommandGroup(new InstantCommand(() -> swerveSubsystem.resetOdometry()), swerveControllerCommand);
    }
    else
    {
      return swerveControllerCommand;
    }
  }
  */




  //trying out the SWERVE CONTROLLER AUTO BUILDER


  /**
   * Use this method to define your button->command mappings. Buttons can be created by
   * instantiating a {@link GenericHID} or one of its subclasses ({@link
   * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then passing it to a {@link
   * edu.wpi.first.wpilibj2.command.button.JoystickButton}.
   */
  private void configureButtonBindings() 
  {
    new JoystickButton(driverJoystick, 2).whenPressed(() -> swerveSubsystem.zeroHeading());
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */



 
//Original getAutonomousCommand
/*   
public Command getAutonomousCommand() 
{
  return autoChooser.getSelected();
} 
*/


//might have to 
public Command getAutonomousCommand() 
{
  String autoName = autoChooser.getSelected().toString();
  PathPlannerTrajectory examplePath;
  examplePath = PathPlanner.loadPath(autoName, new PathConstraints(4, 3));

  // If the path you gave is not in the list, drive forward  
  // Prints for running in simulation, you can comment these our if you want 
  if (examplePath == null) {
    examplePath = PathPlanner.generatePath(
      new PathConstraints(4, 3), 
      new PathPoint(new Translation2d(1.0, 3.0), Rotation2d.fromDegrees(0)), // position, heading
      new PathPoint(new Translation2d(3.0, 3.0), Rotation2d.fromDegrees(0)) // position, heading
  );
  }
  
  System.out.print("========== Starting Auto ==========\n");
  System.out.print("Path: " + autoName + "\n");
  System.out.print("\n\n");

  /**
   *  This HashMap is for all of your manipulator commands tu run durring your autos 
   *  first argument is the command name you set in PathPlanner
   *  second argument is the command in jave you want to run, for this example we are doing a print
   */

  
  HashMap<String, Command> eventMap = new HashMap<>();
  eventMap.put("intakeOn", new PrintCommand("Intake On"));
  eventMap.put("intakeOff", new PrintCommand("Intake Off"));
  eventMap.put("intakeUp", new PrintCommand("Intake Up"));
  eventMap.put("intakeDown", new PrintCommand("Intake Down"));
  eventMap.put("shooterOn", new PrintCommand("SHOOTING!"));
  eventMap.put("shooterOff", new PrintCommand("Shooter Off"));
  eventMap.put("wait", new PrintCommand("Waiting"));


  ArrayList<PathPlannerTrajectory> pathGroup = PathPlanner.loadPathGroup("FullAuto", new PathConstraints(4, 3));


  SwerveAutoBuilder autoBuilder = new SwerveAutoBuilder(
  swerveSubsystem::getPose, 
  swerveSubsystem::resetPoseTo,
  DriveConstants.kDriveKinematics, 
  new PIDConstants(5.0, 0.0, 0.0), 
  new PIDConstants(0.5, 0.0, 0.0),
  swerveSubsystem::setModuleStates, 
  eventMap, 
  swerveSubsystem);

  Command fullAuto = autoBuilder.fullAuto(pathGroup);

  return fullAuto.andThen(() -> swerveSubsystem.stopModules());

} 
}
