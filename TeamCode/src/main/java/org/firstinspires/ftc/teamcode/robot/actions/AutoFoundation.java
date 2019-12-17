package org.firstinspires.ftc.teamcode.robot.actions;

import org.firstinspires.ftc.teamcode.autonomous.Action;
import org.firstinspires.ftc.teamcode.robot.Robot;


public class AutoFoundation implements Action {

    private Robot robot;
    private boolean hold;

    public AutoFoundation(boolean hold) {
        this.hold = hold;
    }

    @Override
    public void setup(Robot robot) {
        this.robot = robot;
    }
    @Override
    public boolean loop() {
        if (hold = true){
            robot.getServoFoundationL().setPosition(0.35);
            robot.getServoFoundationR().setPosition(0.35);
        }else if (hold = false) {
            robot.getServoFoundationL().setPosition(0);
            robot.getServoFoundationR().setPosition(0);
        }
        return hold;
    }
}

