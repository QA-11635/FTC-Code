package org.firstinspires.ftc.teamcode.robot.actions;

import org.firstinspires.ftc.robotcore.external.matrices.OpenGLMatrix;
import org.firstinspires.ftc.robotcore.external.matrices.VectorF;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.robotcore.external.navigation.RelicRecoveryVuMark;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackableDefaultListener;
import org.firstinspires.ftc.teamcode.autonomous.Action;
import org.firstinspires.ftc.teamcode.robot.Robot;
import org.opencv.core.Rect;

import java.util.List;

public class AutoDetect implements Action {

    private Robot robot;

    @Override
    public void setup(Robot robot) {
        this.robot = robot;
    }

    @Override
    public boolean loop() {
        List<Rect> things = robot.getDetector().foundRectangles();
        for (Rect rect: things){
            robot.print("Width", rect.x);
            return true;
        }
        return false;
    }
}
