// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Util;

import com.ctre.phoenix.motorcontrol.can.TalonFX;
import com.ctre.phoenix.motorcontrol.can.TalonFXConfiguration;

import frc.robot.Util.PIDDisplay.PIDSetter;

/** Add your docs here. */
public class TalonFXsetter implements PIDSetter {

    TalonFX motor;
    TalonFXConfiguration config;

    public TalonFXsetter(TalonFX motor, TalonFXConfiguration config){
        this.motor = motor;
        this.config = config;
    }

    @Override
    public void setPID(double p, double i, double d, double f) {
        config.slot0.kP = p;
        config.slot0.kI = i;
        config.slot0.kD = d;
        config.slot0.kF = f;

        motor.configAllSettings(config);



        // motor.config_kP(0, p);
        // motor.config_kI(0, i);
        // motor.config_kD(0, d);
        // motor.config_kF(0, f);
    }


    @Override
    public double[] getPID() {
        double p = config.slot0.kP;
        double i = config.slot0.kI;
        double d = config.slot0.kD;
        double f = config.slot0.kF;

        double[] all = {p, i, d, f};
        return all;
    }
}
