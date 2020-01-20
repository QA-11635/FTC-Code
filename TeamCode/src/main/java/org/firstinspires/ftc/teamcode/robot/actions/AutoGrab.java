package org.firstinspires.ftc.teamcode.robot.actions;

import org.firstinspires.ftc.teamcode.autonomous.Action;
import org.firstinspires.ftc.teamcode.robot.Robot;


public class AutoGrab implements Action {

    private Robot robot;
    private boolean take;

    public AutoGrab(boolean take) {
        this.take = take;
    }

    @Override
    public void setup(Robot robot) {
        this.robot = robot;
    }
    @Override
    public boolean loop() {
        if (take){
            robot.getGrabServo().setPosition(1);
        }else {
            robot.getGrabServo().setPosition(0);
        }
        return take;
    }
}

