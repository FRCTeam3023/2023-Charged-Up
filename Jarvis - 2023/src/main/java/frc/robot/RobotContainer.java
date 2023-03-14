// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.util.HashMap;
import java.util.List;

import com.pathplanner.lib.PathConstraints;
import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.PathPlannerTrajectory;
import com.pathplanner.lib.auto.PIDConstants;
import com.pathplanner.lib.auto.SwerveAutoBuilder;
import com.pathplanner.lib.server.PathPlannerServer;

import org.photonvision.PhotonCamera;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.robot.Constants.ArmConstants;
import frc.robot.Constants.ScoringPositions;
import frc.robot.commands.ArmControl;
import frc.robot.commands.HomeCommand;
import frc.robot.commands.JoystickDrive;
import frc.robot.commands.LevelChargeStation;
import frc.robot.commands.MoveToFieldPosition;
import frc.robot.commands.PercentageControl;
import frc.robot.commands.SetArmState;
import frc.robot.commands.SetClawState;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.Drivetrain;


/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
  // The robot's subsystems and commands are defined here...
  private final PhotonCamera camera = new PhotonCamera("visionCamera");


  private final Drivetrain drivetrain = new Drivetrain(camera);
  private final Arm arm = new Arm();

  private final Joystick mainJoystick = new Joystick(1);
  private final Joystick secondaryJoystick = new Joystick(0);
  
  private final JoystickDrive joystickDrive = new JoystickDrive(drivetrain, mainJoystick);
  private final PercentageControl percentageControl = new PercentageControl(drivetrain, arm, mainJoystick, secondaryJoystick);
  private final HomeCommand homeCommand = new HomeCommand(drivetrain);
  private final ArmControl armControl = new ArmControl(arm, secondaryJoystick);



  // This will load the file "Simple Path.path" and generate it with a max velocity of 2 m/s and a max acceleration of 1 m/s^2
  // for every path in the group
  List<PathPlannerTrajectory> overLineInnerPath = PathPlanner.loadPathGroup("Over Line - Inner", new PathConstraints(3, 1.5));
  List<PathPlannerTrajectory> overLineOuterPath = PathPlanner.loadPathGroup("Over Line - Outer", new PathConstraints(3, 1.5));
  List<PathPlannerTrajectory> balacePath = PathPlanner.loadPathGroup("Balance", new PathConstraints(3, 1.5));
  List<PathPlannerTrajectory> cubeBalance = PathPlanner.loadPathGroup("1 Cube - Balance", new PathConstraints(3, 1.5));
  List<PathPlannerTrajectory> cubeOuter = PathPlanner.loadPathGroup("1 Cube - Outer", new PathConstraints(3, 1.5));
  List<PathPlannerTrajectory> cubeInner = PathPlanner.loadPathGroup("1 Cube - Inner", new PathConstraints(3, 1.5));
  



  // This is just an example event map. It would be better to have a constant, global event map
  // in your code that will be used by all path following commands.
  HashMap<String, Command> eventMap = new HashMap<>();

  
  SendableChooser<List<PathPlannerTrajectory>> autoChooser = new SendableChooser<>();
  


  // Create the AutoBuilder. This only needs to be created once when robot code starts, not every time you want to create an auto command. A good place to put this is in RobotContainer along with your subsystems.
  SwerveAutoBuilder autoBuilder = new SwerveAutoBuilder(
    drivetrain::getRobotPose, // Pose2d supplier
    drivetrain::setCurrentPose, // Pose2d consumer, used to reset odometry at the beginning of auto
    drivetrain.kinematics, // SwerveDriveKinematics
    new PIDConstants(4, 0.0, 0.0), // PID constants to correct for translation error (used to create the X and Y PID controllers)
    new PIDConstants(4, 0.0, 0.0), // PID constants to correct for rotation error (used to create the rotation controller)
    drivetrain::setModuleStates, // Module states consumer used to output to the drive subsystem
    eventMap,
    drivetrain // The drive subsystem. Used to properly set the requirements of path following commands
  );



  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    autoChooser.setDefaultOption("Over Line - Inner", overLineInnerPath);
    autoChooser.addOption("Over Line - Outer", overLineInnerPath);
    autoChooser.addOption("Balance", balacePath);
    autoChooser.addOption("1 Cube - Balance", cubeBalance);
    autoChooser.addOption("1 Cube - Inner", cubeInner);
    autoChooser.addOption("1 Cube - Outer", cubeOuter);
    


    SmartDashboard.putData(autoChooser);

    eventMap.put("Home Modules", new HomeCommand(drivetrain));
    eventMap.put("Neutral State", new SetArmState(arm, ArmConstants.NEUTRAL_STATE));
    eventMap.put("Low State", new SetArmState(arm, ArmConstants.LOW_SCORE_STATE));
    eventMap.put("Mid State", new SetArmState(arm, ArmConstants.MID_SCORE_STATE));
    eventMap.put("High State", new SequentialCommandGroup(
      new SetArmState(arm, ArmConstants.PRE_MID_SCORE_STATE),
      new SetArmState(arm, ArmConstants.HIGH_SCORE_STATE)) );
    eventMap.put("Clearance State", new SetArmState(arm, ArmConstants.CLEARANCE_STATE));
    eventMap.put("Charge Station Balance", new LevelChargeStation(drivetrain));
    eventMap.put("Open Claw", new SetClawState(arm, false));
    eventMap.put("Set Claw Cube", new InstantCommand(() -> arm.resetClawPos(ArmConstants.CUBE_CLAW_OFFSET)));
    eventMap.put("Set Claw Cone", new InstantCommand(() -> arm.resetClawPos(ArmConstants.CONE_CLAW_OFFSET)));


    PathPlannerServer.startServer(5811);
    drivetrain.setDefaultCommand(joystickDrive);
    arm.setDefaultCommand(armControl);




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


    new JoystickButton(mainJoystick, 2).whileTrue(percentageControl);

    new JoystickButton(mainJoystick, 3).whileTrue(homeCommand);

    // new JoystickButton(mainJoystick, 4).onTrue(new InstantCommand(() -> drivetrain.zeroEncoders()).andThen(() -> drivetrain.stopModules()));

    new JoystickButton(mainJoystick, 5).whileTrue(new MoveToFieldPosition(ScoringPositions.Red2, drivetrain));
    new JoystickButton(mainJoystick, 6).whileTrue(new MoveToFieldPosition(ScoringPositions.RED_PICKUP_RIGHT, drivetrain));


    //zero Gyro angle, counter drift during testing. Hopefully get a better gyro soon  (Will make a loop overrun warning)
    new JoystickButton(mainJoystick, 7).onTrue(new InstantCommand(() -> drivetrain.calibrateGyro()));
    new JoystickButton(mainJoystick, 12).onTrue(new InstantCommand(() -> arm.zeroEncoders()));

    new JoystickButton(mainJoystick, 4).whileTrue(new MoveToFieldPosition(new Pose2d(14, 3, new Rotation2d()), drivetrain));


    /*------------------------------------------------------------------------------------------- */


    // new JoystickButton(secondaryJoystick, 1).whileTrue(new SetClawState(arm, true));
    // new JoystickButton(secondaryJoystick, 2).whileTrue(new SetClawState(arm, false));



    new JoystickButton(secondaryJoystick, 3).whileTrue(new SetArmState(arm, ArmConstants.NEUTRAL_STATE));

    new JoystickButton(secondaryJoystick, 5).whileTrue(
      new SequentialCommandGroup(
        new SetArmState(arm, ArmConstants.CLEARANCE_STATE),
        new SetArmState(arm, ArmConstants.PICKUP_STATE))
    );

    new JoystickButton(secondaryJoystick, 4).whileTrue(
      new SequentialCommandGroup(
        new SetArmState(arm, ArmConstants.CLEARANCE_STATE),
        new SetArmState(arm, ArmConstants.MID_SCORE_STATE))
    );

    new JoystickButton(secondaryJoystick, 6).whileTrue( 
      new SequentialCommandGroup(
        new SetArmState(arm, ArmConstants.PRE_MID_SCORE_STATE),
        new SetArmState(arm, ArmConstants.HIGH_SCORE_STATE)) 
    );


    new JoystickButton(secondaryJoystick, 8).whileTrue(new InstantCommand(()->arm.setClawMotorOutput(0.2)).andThen(() -> arm.setClawMotorOutput(0)));
    new JoystickButton(secondaryJoystick, 10).whileTrue(new InstantCommand(()->arm.setClawMotorOutput(-0.2)).andThen(() -> arm.setClawMotorOutput(0)));


    new JoystickButton(secondaryJoystick, 7).onTrue(new InstantCommand(() -> arm.zeroClawPos()));


    
    

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

    List<PathPlannerTrajectory> selectedPath = autoChooser.getSelected();

    Command fullAuto = autoBuilder.fullAuto(selectedPath);

    return fullAuto;

  }

  
}
