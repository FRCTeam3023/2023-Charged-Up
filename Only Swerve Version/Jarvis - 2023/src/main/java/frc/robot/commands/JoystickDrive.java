// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import frc.robot.Constants;
import frc.robot.Constants.ModuleConstants;
import frc.robot.Util.PIDDisplay;
import frc.robot.Util.WPILibSetter;
import frc.robot.subsystems.Drivetrain;

import java.util.List;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj2.command.CommandBase;

/** An example command that uses an example subsystem. */
public class JoystickDrive extends CommandBase {
  private final Drivetrain drivetrain;
  // private final Joystick leftJoystick;
  // private final Joystick rightJoystick;
  private final Joystick controller;

  private double xInputRight;
  private double yInputRight;
  private double joystickAngle;
  private double joystickMagnitude;


  private static PIDController rotationController = new PIDController(6, 0, 0);

  public WPILibSetter turnPIDSetter = new WPILibSetter(List.of(rotationController));


  private double targetRotation;

  /**
   * Creates a new ExampleCommand.
   *
   * @param subsystem The subsystem used by this command.
   */
  public JoystickDrive(Drivetrain drivetrain, Joystick controller) {
    this.drivetrain = drivetrain;
    this.controller = controller; 
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(drivetrain);
    rotationController.enableContinuousInput(-Math.PI, Math.PI);
    PIDDisplay.PIDList.addOption("Chassis Turn", turnPIDSetter);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {

    if(Constants.USE_CONTROLLER){
      xInputRight = -applyDeadband(controller.getRawAxis(4), Constants.DRIVE_TOLERANCE_PERCENT) * 2;
      yInputRight = -applyDeadband(controller.getRawAxis(5), Constants.DRIVE_TOLERANCE_PERCENT) * 2;

      joystickAngle = Math.atan2(-controller.getRawAxis(1), controller.getRawAxis(0)) - Math.PI/2;
      System.out.println(joystickAngle);
      joystickMagnitude = Math.sqrt(Math.pow(controller.getRawAxis(0), 2) + Math.pow(controller.getRawAxis(1), 2)); 
    }else{
      // xInputRight = applyDeadband(rightJoystick.getX(), Constants.DRIVE_TOLERANCE_PERCENT);
      // yInputRight = applyDeadband(rightJoystick.getY(), Constants.DRIVE_TOLERANCE_PERCENT);

      // joystickAngle = Math.atan2(leftJoystick.getX(), -leftJoystick.getY());
      // joystickMagnitude = Math.sqrt(Math.pow(leftJoystick.getX(), 2) + Math.pow(leftJoystick.getY(), 2));
    }
    

    if (joystickMagnitude > 0.25) {
      targetRotation = joystickAngle;
    }else{
      targetRotation = drivetrain.getChassisAngle().getRadians();
    }

    double rotationSpeed = rotationController.calculate(drivetrain.getChassisAngle().getRadians(), targetRotation);    

    drivetrain.drive(yInputRight, xInputRight, rotationSpeed, true);
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
