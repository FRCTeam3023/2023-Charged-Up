// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.util.HashMap;


import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.commands.PathPlannerAuto;
import com.pathplanner.lib.util.HolonomicPathFollowerConfig;
import com.pathplanner.lib.util.PIDConstants;
import com.pathplanner.lib.util.ReplanningConfig;

import org.photonvision.PhotonCamera;

import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.robot.Constants.ModuleConstants;
import frc.robot.Constants.ScoringPositions;
import frc.robot.Util.PIDDisplay;
import frc.robot.commands.HomeCommand;
import frc.robot.commands.JoystickDrive;
import frc.robot.commands.LevelChargeStation;
import frc.robot.commands.MoveToFieldPosition;
import frc.robot.subsystems.Drivetrain;


/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
  public static final String PIDList = null;


  // The robot's subsystems and commands are defined here...
  private final PhotonCamera camera = new PhotonCamera("visionCamera");


  private final Drivetrain drivetrain = new Drivetrain(camera);

  // private final Joystick leftJoystick = new Joystick(0);
  // private final Joystick rightJoystick = new Joystick(1);
  // private final Joystick launchpad = new Joystick(2);
  private final Joystick controller = new Joystick(3);
  
  
  private final JoystickDrive joystickDrive = new JoystickDrive(drivetrain, controller);
  private final HomeCommand homeCommand = new HomeCommand(drivetrain);


  




  // This will load the file "Simple Path.path" and generate it with a max velocity of 2 m/s and a max acceleration of 1 m/s^2
  // for every path in the group
  // List<PathPlannerTrajectory> overLineInnerPath = PathPlanner.loadPathGroup("Over Line - Inner", new PathConstraints(3, 1.5));

  // List<PathPlannerTrajectory> overLineOuterPath = PathPlanner.loadPathGroup("Over Line - Outer", new PathConstraints(3, 1.5));
  // List<PathPlannerTrajectory> balacePath = PathPlanner.loadPathGroup("Balance", new PathConstraints(3, 1.5));
  // List<PathPlannerTrajectory> cubeBalance = PathPlanner.loadPathGroup("1 Cube - Balance", new PathConstraints(3, 1.5));
  // List<PathPlannerTrajectory> cubeOuter = PathPlanner.loadPathGroup("1 Cube - Outer", new PathConstraints(3, 1.5));
  // List<PathPlannerTrajectory> cubeInner = PathPlanner.loadPathGroup("1 Cube - Inner", new PathConstraints(3, 1.5));

  // List<PathPlannerTrajectory> overLineInnerRed = PathPlanner.loadPathGroup("Across Line - Inner Red", new PathConstraints(3, 1.5));
  // List<PathPlannerTrajectory> overLineOuterRed = PathPlanner.loadPathGroup("Across Line - Outer Red", new PathConstraints(3, 1.5));
  // List<PathPlannerTrajectory> BalanceRed = PathPlanner.loadPathGroup("Balance Red", new PathConstraints(3, 1.5));

  // List<PathPlannerTrajectory> cubeBalanceRed = PathPlanner.loadPathGroup("1 Cube - Balance Red", new PathConstraints(3, 1.5));
  // List<PathPlannerTrajectory> cubeInnerRed = PathPlanner.loadPathGroup("1 Cube - Inner Red", new PathConstraints(3, 1.5));
  // List<PathPlannerTrajectory> cubeOuterRed = PathPlanner.loadPathGroup("1 Cube - Outer Red", new PathConstraints(3, 1.5));







  // This is just an example event map. It would be better to have a constant, global event map
  // in your code that will be used by all path following commands.
  HashMap<String, Command> eventMap = new HashMap<>();

  
  SendableChooser<String> autoChooser = new SendableChooser<>();
  
  


  



  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {

    new PIDDisplay();

    autoChooser.setDefaultOption("Over Line - Inner", "Over Line - Inner");
    autoChooser.addOption("Over Line - Outer", "Over Line - Outer");
    autoChooser.addOption("Balance", "Balance");

    autoChooser.addOption("Cube - Balance", "1 Cube - Balance");
    autoChooser.addOption("Cube - Inner", "1 Cube - Inner");
    autoChooser.addOption("Cube - Outer", "1 Cube - Outer");

    autoChooser.addOption("Over Line - Outer Red", "Across Line - Inner Red");
    autoChooser.addOption("Over Line - Inner Red", "Across Line - Outer Red");
    autoChooser.addOption("Balance Red", "Balance Red");

    autoChooser.addOption("Cube - Balance Red", "1 Cube - Balance Red");
    autoChooser.addOption("Cube - Inner Red", "1 Cube - Inner Red");
    autoChooser.addOption("Cube - Outer Red", "1 Cube - Outer Red");





    

    AutoBuilder.configureHolonomic(
      drivetrain::getRobotPose,
      drivetrain::setCurrentPose,
      drivetrain::getRobotRelativeSpeeds, 
      drivetrain::drive, 
      new HolonomicPathFollowerConfig(
        new PIDConstants(4), 
        new PIDConstants(4), 
        ModuleConstants.MAX_SPEED, 
        drivetrain.frontLeftLocation.getX() * Math.sqrt(2) , 
        new ReplanningConfig()), drivetrain);

      


    SmartDashboard.putData(autoChooser);

    eventMap.put("Home Modules", new HomeCommand(drivetrain));
    eventMap.put("Charge Station Balance", new LevelChargeStation(drivetrain));

    drivetrain.setDefaultCommand(joystickDrive);




    // Configure the button bindings
    configureButtonBindings();
  }

  /**
   * Use this method to define your button->command mappings. Buttons can be created by
   * instantiating a {@link GenericHID} or one of its subclasses ({@link
   * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then passing it to a {@link
   * edu.wpi.first.wpilibj2.command.button.JoystickButton}.
   */
  private void configureButtonBindings() {
    /*
     * Current Button Binding layout:
     * 
     * 
     * 1: Position Control for arm base joint
     * 2: Home Modules
     * 3: 0 Module encoders - both motors
     * 7: Recalibrate Gyro
     * 
     * 
     * 
     */


    // new JoystickButton(rightJoystick, 2).whileTrue(percentageControl);


    // new JoystickButton(rightJoystick, 3).whileTrue(homeCommand);
    new JoystickButton(controller, 1).whileTrue(homeCommand);



    // //zero Gyro angle, counter drift during testing. Hopefully get a better gyro soon  (Will make a loop overrun warning)
    // new JoystickButton(rightJoystick, 7).onTrue(new InstantCommand(() -> drivetrain.calibrateGyro()));

    // new JoystickButton(launchpad, ScoringPositions.SCORING_BUTTON_1).whileTrue(new MoveToFieldPosition(1, drivetrain));
    // new JoystickButton(launchpad, ScoringPositions.SCORING_BUTTON_2).whileTrue(new MoveToFieldPosition(2, drivetrain));
    // new JoystickButton(launchpad, ScoringPositions.SCORING_BUTTON_3).whileTrue(new MoveToFieldPosition(3, drivetrain));
    // new JoystickButton(launchpad, ScoringPositions.SCORING_BUTTON_4).whileTrue(new MoveToFieldPosition(4, drivetrain));
    // new JoystickButton(launchpad, ScoringPositions.SCORING_BUTTON_5).whileTrue(new MoveToFieldPosition(5, drivetrain));
    // new JoystickButton(launchpad, ScoringPositions.SCORING_BUTTON_6).whileTrue(new MoveToFieldPosition(6, drivetrain));
    // new JoystickButton(launchpad, ScoringPositions.SCORING_BUTTON_7).whileTrue(new MoveToFieldPosition(7, drivetrain));
    // new JoystickButton(launchpad, ScoringPositions.SCORING_BUTTON_8).whileTrue(new MoveToFieldPosition(8, drivetrain));
    // new JoystickButton(launchpad, ScoringPositions.SCORING_BUTTON_9).whileTrue(new MoveToFieldPosition(9, drivetrain));

    // new JoystickButton(rightJoystick, 11).whileTrue(new MoveToFieldPosition(10, drivetrain));
    // new JoystickButton(rightJoystick, 12).whileTrue(new MoveToFieldPosition(11, drivetrain));
  }

  public Drivetrain getDrivetrain(){
    return drivetrain;
  }


  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {

    String selectedPath = autoChooser.getSelected();


    //Command fullAuto = autoBuilder.fullAuto(selectedPath);

    Command fullAuto = new PathPlannerAuto(selectedPath);


    return fullAuto;

  }

  
}
