package org.firstinspires.ftc.teamcode.robot.actions;

import org.firstinspires.ftc.teamcode.autonomous.Action;
import org.firstinspires.ftc.teamcode.robot.Robot;


public class AutoGrab implements Action {

    private Robot robot;
    private int take = 0;

    public AutoGrab(int take) {
        this.take = take;
    }

    @Override
    public void setup(Robot robot) {
        this.robot = robot;
    }
    @Override
    public boolean loop() {
        if (take == 1){
            robot.getGrabServo().setPosition(1);
        }else if (take == 0) {
            robot.getGrabServo().setPosition(0);
        }
        return true;
    }
}

