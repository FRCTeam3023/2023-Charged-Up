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

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.robot.commands.HoldState;
import frc.robot.commands.HomeCommand;
import frc.robot.commands.JoystickDrive;
import frc.robot.commands.PercentageControl;
import frc.robot.commands.PositionControl;
import frc.robot.commands.SetArmState;
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
  // private final PhotonCamera camera = new PhotonCamera("visionCamera");

  private final Drivetrain drivetrain = new Drivetrain(/*camera*/);
  private final Arm arm = new Arm();

  private final Joystick mainJoystick = new Joystick(1);
  private final Joystick secondaryJoystick = new Joystick(0);
  
  private final JoystickDrive joystickDrive = new JoystickDrive(drivetrain, mainJoystick);
  private final PositionControl positionControl = new PositionControl(drivetrain, arm, mainJoystick);
  private final PercentageControl percentageControl = new PercentageControl(drivetrain, arm, mainJoystick, secondaryJoystick);
  private final HoldState holdState = new HoldState(arm);


  // This will load the file "Simple Path.path" and generate it with a max velocity of 2 m/s and a max acceleration of 1 m/s^2
  // for every path in the group
  List<PathPlannerTrajectory> overLineInnerPath = PathPlanner.loadPathGroup("Over Line - Inner", new PathConstraints(3, 1.5));
  List<PathPlannerTrajectory> overLineOuterPath = PathPlanner.loadPathGroup("Over Line - Outer", new PathConstraints(3, 1.5));


  // This is just an example event map. It would be better to have a constant, global event map
  // in your code that will be used by all path following commands.
  HashMap<String, Command> eventMap = new HashMap<>();
  eventMap.put("Home Modules", new HomeCommand(drivetrain));

  

  


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
    SmartDashboard.putBoolean("triggered", false);
    eventMap.put("event", new InstantCommand(() -> SmartDashboard.putBoolean("triggered", true)));
    eventMap.put("deployIntake", new HomeCommand(drivetrain));

    PathPlannerServer.startServer(5811);
    drivetrain.setDefaultCommand(joystickDrive);
    arm.setDefaultCommand(holdState);

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


    new JoystickButton(mainJoystick, 1).whileTrue(positionControl);

    new JoystickButton(mainJoystick, 2).whileTrue(percentageControl);

    new JoystickButton(mainJoystick, 3).whileTrue(new HomeCommand(drivetrain));

    new JoystickButton(mainJoystick, 4).onTrue(new InstantCommand(() -> drivetrain.zeroEncoders()).andThen(() -> drivetrain.stopModules()));


    //zero Gyro angle, counter drift during testing. Hopefully get a better gyro soon  (Will make a loop overrun warning)
    new JoystickButton(mainJoystick, 7).onTrue(new InstantCommand(() -> drivetrain.calibrateGyro()));

    new JoystickButton(mainJoystick, 12).onTrue(new InstantCommand(() -> arm.zeroEncoders()));

    new JoystickButton(mainJoystick, 5).onTrue(new InstantCommand(() -> arm.setClawMotorOutput(0.25)).andThen(() -> arm.stopAllMotors()));

    new JoystickButton(secondaryJoystick, 4).onTrue(new SetArmState(arm, new ArmState()));


    
    

  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {

    // Command fullAuto = autoBuilder.fullAuto(pathGroup);

    return new SequentialCommandGroup(
      new SetArmState(arm, new ArmState()),
      new SetArmState(arm, new ArmState(new Rotation2d(),new Rotation2d(Math.PI/4),new Rotation2d(),0)),
      new SetArmState(arm, new ArmState(new Rotation2d(Math.PI/4),new Rotation2d(Math.PI/4),new Rotation2d(Math.PI/4),0)),
      new SetArmState(arm, new ArmState(new Rotation2d(), new Rotation2d(Math.PI/4), new Rotation2d(), 0)),
      new SetArmState(arm, new ArmState())


    );

  }

  
}
