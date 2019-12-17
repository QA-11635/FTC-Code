package org.firstinspires.ftc.teamcode.robot.actions;

import org.firstinspires.ftc.teamcode.autonomous.Action;
import org.firstinspires.ftc.teamcode.robot.Robot;


public class AutoLazySusan implements Action {

    private Robot robot;
    private boolean spin;

    public AutoLazySusan(boolean spin) {
        this.spin = spin;
    }

    @Override
    public void setup(Robot robot) {
        this.robot = robot;
    }
    @Override
    public boolean loop() {
        if (spin = true){
            robot.getLazySusan().setPosition(0.35);
        }else if (spin = false) {
            robot.getLazySusan().setPosition(0);
        }
        return spin;
    }
}

