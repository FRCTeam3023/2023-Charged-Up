// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Util;

import com.revrobotics.SparkMaxPIDController;

import frc.robot.Util.PIDDisplay.PIDSetter;

/** Add your docs here. */
public class SparkMaxSetter implements PIDSetter {

    private SparkMaxPIDController motor;
    
    public SparkMaxSetter(SparkMaxPIDController motor){
        this.motor = motor;
    }

    @Override
    public void setPID(double p, double i, double d, double f) {

        motor.setP(p);
        motor.setI(i);
        motor.setD(d);
        motor.setFF(f);

        
    }

    @Override
    public double[] getPID() {
        double p = motor.getP();
        double i = motor.getI();
        double d = motor.getD();
        double f = motor.getFF();

        double[] all = {p, i, d, f};
        return all;
    }

    

    
}
