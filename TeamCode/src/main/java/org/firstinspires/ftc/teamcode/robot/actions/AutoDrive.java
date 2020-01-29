package org.firstinspires.ftc.teamcode.robot.actions;

import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.teamcode.autonomous.Action;
import org.firstinspires.ftc.teamcode.robot.Robot;
import org.firstinspires.ftc.teamcode.robot.actions.AutoDetect;

public class AutoDrive implements Action {

    private Robot robot;

//    private double initial = 0;
    public int setPointMeters;
    public double speed;

    public AutoDrive(int encoder, double speed) {
        this.setPointMeters = encoder;
        this.speed = speed;
    }

    @Override
    public void setup(Robot robot) {
        this.robot = robot;
//        this.initial = robot.getRightB().getCurrentPosition();

    }

    @Override
    public boolean loop() {
//        double power = robot.getDrivePID().distance(robot.getLeftB().getCurrentPosition(),   Robot.TICK_PER_METER_DRIVE * setPointMeters, 1);
//        robot.drive(power, 0);

//        robot.print("power", power);
//        robot.print("encoder", robot.getRightB().getCurrentPosition());
////        robot.print("inital", initial + Robot.TICK_PER_METER_DRIVE * setPointMeters);
////        robot.print("inital1", initial);
//        robot.print("ticks", Robot.TICK_PER_METER_DRIVE);
//        robot.print("setPoint", setPointMeters);

//        int distance = 3500 * setPointMeters;
        robot.getLeftF().setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.getLeftB().setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.getRightF().setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.getRightB().setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        robot.getLeftF().setTargetPosition(setPointMeters);
        robot.getLeftB().setTargetPosition(setPointMeters);
        robot.getRightF().setTargetPosition(setPointMeters);
        robot.getRightB().setTargetPosition(setPointMeters);

        robot.getLeftF().setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robot.getLeftB().setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robot.getRightF().setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robot.getRightB().setMode(DcMotor.RunMode.RUN_TO_POSITION);

        robot.getLeftF().setPower(speed);
        robot.getLeftB().setPower(speed);
        robot.getRightF().setPower(-speed);
        robot.getRightB().setPower(-speed);



        while (robot.getLeftF().isBusy() && robot.getLeftB().isBusy() && robot.getRightF().isBusy() && robot.getRightB().isBusy()){

        }

        robot.getLeftF().setPower(0);
        robot.getLeftB().setPower(0);
        robot.getRightF().setPower(0);
        robot.getRightB().setPower(0);

        robot.getLeftF().setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.getLeftB().setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.getRightF().setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.getRightB().setMode(DcMotor.RunMode.RUN_USING_ENCODER);


        return true;


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