package org.firstinspires.ftc.teamcode.robot.actions;

import org.firstinspires.ftc.teamcode.autonomous.Action;
import org.firstinspires.ftc.teamcode.robot.Robot;

public class AutoDrive implements Action {

    private Robot robot;
    private double initial = 0;

    private double setPointMeters;

    public AutoDrive(double meters) {
        this.setPointMeters = meters;
    }

    @Override
    public void setup(Robot robot) {
        this.robot = robot;
        this.initial = robot.getLeftB().getCurrentPosition();
    }

    @Override
    public boolean loop() {
        double power = robot.getDrivePID().distance(robot.getLeftB().getCurrentPosition(), initial + Robot.TICK_PER_METER * setPointMeters, 1);
        robot.drive(power, 0);
        return Math.abs(power) == 0;
    }
}