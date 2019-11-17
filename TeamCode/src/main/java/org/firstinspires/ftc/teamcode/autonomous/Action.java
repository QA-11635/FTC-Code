package org.firstinspires.ftc.teamcode.autonomous;

import org.firstinspires.ftc.teamcode.robot.Robot;

public interface Action {
    void setup(Robot robot);
    boolean loop();
}