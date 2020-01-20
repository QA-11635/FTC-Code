package org.firstinspires.ftc.teamcode.robot.actions;


import org.firstinspires.ftc.teamcode.autonomous.Action;
import org.firstinspires.ftc.teamcode.robot.Robot;
import org.firstinspires.ftc.teamcode.robot.actions.AutoTurn;


public class AutoDetect implements Action {

    private Robot robot;
    private boolean on;

    public AutoDetect(boolean on) {
        this.on = on;
    }

    @Override
    public void setup(Robot robot) {
        this.robot = robot;
    }
    @Override
    public boolean loop() {
        if (on){
        }
            if (robot.getColorSensorL().alpha() >= 50 && robot.getColorSensorL().alpha() <= 62){
                new AutoTurn(-90);
                robot.getCollectL().setPower(1);
                robot.getCollectR().setPower(1);
                new AutoDrive(50);
                return on = true;
            }
            if (robot.getColorSensorL().alpha() >= 99 && robot.getColorSensorL().alpha() <= 122){
                robot.getCollectL().setPower(0);
                robot.getCollectR().setPower(0);
                return on = false;
            }return on;
    }
}