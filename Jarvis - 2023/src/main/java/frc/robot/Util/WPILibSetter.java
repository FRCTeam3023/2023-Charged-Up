// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Util;

import edu.wpi.first.math.controller.PIDController;
import frc.robot.Util.PIDDisplay.PIDSetter;

/** Add your docs here. */
public class WPILibSetter implements PIDSetter{
    private PIDController controller;

    public WPILibSetter(PIDController controller){
        this.controller = controller;
    }

    @Override
    public void setPID(double p, double i, double d, double f) {
        controller.setPID(p, i, d);
    }

    @Override
    public double[] getPID() {
        double p = controller.getP();
        double i = controller.getI();
        double d = controller.getD();
        double f = 0;

        double[] all = {p, i, d, f};
        return all;
    }


}
