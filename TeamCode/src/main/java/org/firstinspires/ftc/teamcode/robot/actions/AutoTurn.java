package org.firstinspires.ftc.teamcode.robot.actions;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.teamcode.PID;
import org.firstinspires.ftc.teamcode.autonomous.Action;
import org.firstinspires.ftc.teamcode.robot.Robot;

public class AutoTurn implements Action {

    private Robot robot;

    private double setPoint;
    private double globalAngle;

    public AutoTurn(double setPoint) {
        this.setPoint = setPoint;
    }

    @Override
    public void setup(Robot robot) {
        this.robot = robot;
    }

    @Override
    public boolean loop() {
        double fix = robot.getTurnPID().angle(robot.getLastAngles().firstAngle, setPoint, 2);
        double drive = 0;
        double turn = -fix;
        turn /= 2;
        robot.drive(drive, turn);
        robot.print("Deg", robot.getLastAngles().firstAngle);
        robot.print("fix", fix);
        robot.print("Counter degrees", globalAngle);
        Orientation angles = robot.update();
        globalAngle += (angles.firstAngle - robot.getLastAngles().firstAngle + 180) % 360 - 180;
        robot.setLastAngles(angles);
        return Math.abs(turn) < 0.02;
    }
}