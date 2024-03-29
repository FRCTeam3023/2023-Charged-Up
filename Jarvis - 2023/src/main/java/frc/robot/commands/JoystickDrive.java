// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import frc.robot.Constants;
import frc.robot.subsystems.Drivetrain;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj2.command.CommandBase;

/** An example command that uses an example subsystem. */
public class JoystickDrive extends CommandBase {
  private final Drivetrain drivetrain;
  private final Joystick joystick;

  private final PIDController rotationController = new PIDController(0, 0, 0);

  private double xInput;
  private double yInput;

  /**
   * Creates a new ExampleCommand.
   *
   * @param subsystem The subsystem used by this command.
   */
  public JoystickDrive(Drivetrain drivetrain, Joystick joystick) {
    this.drivetrain = drivetrain;
    this.joystick = joystick;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(drivetrain);
    rotationController.enableContinuousInput(-Math.PI, Math.PI);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    //take joystick inputs scaled to the max speed of the bot

    

    xInput = applyDeadband(-joystick.getRawAxis(4), Constants.DRIVE_TOLERANCE_PERCENT);
    yInput = applyDeadband(-joystick.getRawAxis(5), Constants.DRIVE_TOLERANCE_PERCENT);

    double joystickAngle = Math.atan2(joystick.getRawAxis(0), -joystick.getRawAxis(1));
    // rInput = applyDeadband(-joystick.getTwist(), Constants.ROTATION_TOLERANCE_PERCENT);.


    // if(joystick.getRawButton(1)){
    //   xSpeed = yInput * ModuleConstants.SLOW_MAX_SPEED;
    //   ySpeed = xInput * ModuleConstants.SLOW_MAX_SPEED;
    //   rot = rInput * Constants.SLOW_MAX_ANGULAR_SPEED;
    // }else{
      
    //   xSpeed = yInput * ModuleConstants.MAX_SPEED;
    //   ySpeed = xInput * ModuleConstants.MAX_SPEED;
    //   rot = rInput * Constants.MAX_ANGULAR_SPEED;
    // }

    // if(joystick.getPOV() != -1){
    //   xSpeed = Math.cos(joystick.getPOV() * (Math.PI/180)) * 0.2;
    //   ySpeed = -Math.sin(joystick.getPOV() * (Math.PI/180)) * 0.2;
    // }
    
    double rot = Math.sqrt(Math.pow(joystick.getRawAxis(0), 2) + Math.pow(joystick.getRawAxis(1), 2)) * rotationController.calculate(drivetrain.getRobotPose().getRotation().getRadians(), joystickAngle);

    drivetrain.drive(xInput, yInput, rot, true);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    drivetrain.stopModules();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }


  private double applyDeadband(double joystickValue, double tolerance){
    if(Math.abs(joystickValue) > tolerance){
      return Math.signum(joystickValue) * (Math.abs(joystickValue) - tolerance)/(1-tolerance);
    } 
    return 0;
  }
}
