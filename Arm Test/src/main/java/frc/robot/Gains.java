package frc.robot;

public class Gains {
    
    public double P;
    public double I;
    public double D;
    public double F;
    public double maxOutput;
    

    public Gains(double P, double I, double D, double F, double maxOutput){
        this.P = P;
        this.I = I;
        this.D = D;
        this.F = F;
        this.maxOutput = maxOutput;
    }
}
