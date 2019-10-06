package org.firstinspires.ftc.teamcode;

public class PID {

    private static final double DT = 0.01;

    private double kP, kI, kD;
    private double lastError = 0;
    private double integral = 0, derivative = 0;

    public PID(double kP, double kI, double kD) {
        this.kP = kP;
        this.kI = kI;
        this.kD = kD;
    }

    public double calculate(double current, double setpoint) {
        double error = setpoint - current;
        derivative = (error - lastError) / DT;
        integral += error * DT;
        return kP * (error) + kI * (integral) + kD * (derivative);
    }
}
