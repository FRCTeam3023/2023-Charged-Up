// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.TalonFXFeedbackDevice;
import com.ctre.phoenix.motorcontrol.can.TalonFX;

import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Gains;
import frc.robot.Constants.ArmConstants;

public class Arm extends SubsystemBase {

  private TalonFX baseMotor = new TalonFX(20);

  public static double baseSpeedLimit = 0.25;

  public Gains baseJointGains = new Gains(0, 0, 0, 0, 1);



  public Arm() {
    baseMotor.configFactoryDefault();
    baseMotor.setNeutralMode(NeutralMode.Brake);
    baseMotor.set(ControlMode.PercentOutput, 0);
    baseMotor.setInverted(false);

    //select internal sensor
    baseMotor.configSelectedFeedbackSensor(TalonFXFeedbackDevice.IntegratedSensor, Constants.PRIMARY_PID_LOOP_IDX, Constants.TIMEOUT_MS);
    //change the feedback units to degrees of output
    baseMotor.configSelectedFeedbackCoefficient( (1/Constants.FALCON_UNITS_PER_REV) * (1/ArmConstants.BASE_GEAR_RATIO) * 360);
    

    baseMotor.config_kP(Constants.PRIMARY_PID_LOOP_IDX, baseJointGains.P);
    baseMotor.config_kI(Constants.PRIMARY_PID_LOOP_IDX, baseJointGains.I);
    baseMotor.config_kD(Constants.PRIMARY_PID_LOOP_IDX, baseJointGains.D);
    baseMotor.config_kF(Constants.PRIMARY_PID_LOOP_IDX, baseJointGains.F);



    baseMotor.setSelectedSensorPosition(0);
  }

  /**
   * Example command factory method.
   *
   * @return a command
   */
  public CommandBase exampleMethodCommand() {
    // Inline construction of command goes here.
    // Subsystem::RunOnce implicitly requires `this` subsystem.
    return runOnce(
        () -> {
          /* one-time action goes here */
        });
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  @Override
  public void simulationPeriodic() {
    // This method will be called once per scheduler run during simulation
  }

  /**
   * Sets the base joint to a specific percent output
   * @param percentOutput %
   */
  public void setBaseMotorOutput(double percentOutput){
    baseMotor.set(ControlMode.PercentOutput, percentOutput);
  }

  public void setBaseMotorPosition(double angle){
    baseMotor.set(ControlMode.Position, angle);
  }
}
