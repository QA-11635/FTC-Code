//package org.firstinspires.ftc.teamcode.robot.actions;
//
//import org.firstinspires.ftc.teamcode.autonomous.Action;
//import org.firstinspires.ftc.teamcode.robot.Robot;
//
//
//public class AutoFoundation implements Action {
//
//    private Robot robot;
//    private boolean hold;
//
//    public AutoFoundation(boolean hold) {
//        this.hold = hold;
//    }
//
//    @Override
//    public void setup(Robot robot) {
//        this.robot = robot;
//    }
//    @Override
//    public boolean loop() {
//        if (hold){
//            robot.getServoFoundationL().setPosition(0.4);
//            robot.getServoFoundationR().setPosition(0.4);
//            if (robot.getServoFoundationL().getPosition() >= 0.3 && robot.getServoFoundationR().getPosition() >= 0.3){
//                robot.getServoFoundationL().setPosition(0);
//                robot.getServoFoundationR().setPosition(0);
//            }
//        }else {
//            robot.getServoFoundationL().setPosition(0);
//            robot.getServoFoundationR().setPosition(0);
//        }
//        return hold;
//    }
//}
//
