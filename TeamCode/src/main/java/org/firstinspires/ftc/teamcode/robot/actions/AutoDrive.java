package org.firstinspires.ftc.teamcode.robot.actions;

import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.teamcode.autonomous.Action;
import org.firstinspires.ftc.teamcode.robot.Robot;

public class AutoDrive implements Action {

    private Robot robot;

//    private double initial = 0;
    private double setPointMeters;

    public AutoDrive(double meters) {
        this.setPointMeters = meters;
    }

    @Override
    public void setup(Robot robot) {
        this.robot = robot;
//        this.initial = robot.getRightB().getCurrentPosition();

    }

    @Override
    public boolean loop() {
        double power = robot.getDrivePID().distance(robot.getRightB().getCurrentPosition(),   Robot.TICK_PER_METER_DRIVE * setPointMeters, 1);
        robot.drive(power, 0);

        robot.print("power", power);
        robot.print("encoder", robot.getRightB().getCurrentPosition());
//        robot.print("inital", initial + Robot.TICK_PER_METER_DRIVE * setPointMeters);
//        robot.print("inital1", initial);
        robot.print("ticks", Robot.TICK_PER_METER_DRIVE);
        robot.print("setPoint", setPointMeters);

        robot.getLeftF().setPower(power);
        robot.getLeftB().setPower(power);
        robot.getRightF().setPower(power);
        robot.getRightB().setPower(power);


        return Math.abs(power) == 0;


//        double fix = robot.getDrivePID().distance(robot.getRightB().getCurrentPosition(), initial + setPointMeters * Robot.TICK_PER_METER_DRIVE, 1);
//        setPointMeters = initial + setPointMeters * Robot.TICK_PER_METER_DRIVE;
//        double drive = -fix;
//        double turn = -0;
//        drive /= 2;
//        robot.drive(drive, turn);
//        robot.print("meter", robot.getRightB().getCurrentPosition());
//        robot.print("fix", fix);
//        robot.print("encoder", robot.getRightB().getCurrentPosition());
//        initial += initial - robot.getRightB().getCurrentPosition();
//        return Math.abs(fix) == 0;
    }
}