/* Copyright (c) 2017 FIRST. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted (subject to the limitations in the disclaimer below) provided that
 * the following conditions are met:
 *
 * Redistributions of source code must etain the above copyright notice, this list
 * of conditions and the following disclaimer.
 *
 * Redistributions in binary form must reproduce the above copyright notice, this
 * list of conditions and the following disclaimer in the documentation and/or
 * other materials provided with the distribution.
 *
 * Neither the name of FIRST nor the names of its contributors may be used to endorse or
 * promote products derived from this software without specific prior written permission.
 *
 * NO EXPRESS OR IMPLIED LICENSES TO ANY PARTY'S PATENT RIGHTS ARE GRANTED BY THIS
 * LICENSE. THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
 * THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

package org.firstinspires.ftc.teamcode.SkyStoneRobot;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.teamcode.autonomous.Action;
import org.firstinspires.ftc.teamcode.robot.Robot;
import org.firstinspires.ftc.teamcode.robot.actions.AutoCollect;
import org.firstinspires.ftc.teamcode.robot.actions.AutoDetect;
import org.firstinspires.ftc.teamcode.robot.actions.AutoGrab;
import org.firstinspires.ftc.teamcode.robot.actions.AutoHex;
import org.firstinspires.ftc.teamcode.robot.actions.AutoLift;
//import org.firstinspires.ftc.teamcode.robot.actions.AutoLine;
import org.firstinspires.ftc.teamcode.robot.actions.AutoTurn;
import org.firstinspires.ftc.teamcode.robot.actions.AutoDrive;
import org.firstinspires.ftc.teamcode.robot.actions.AutoFoundation;


/**
 * This file contains an example of an iterative (Non-Linear) "OpMode".
 * An OpMode is a 'program' that runs in either the autonomous or the teleop period of an FTC match.
 * The names of OpModes appear on the menu of the FTC Driver Station.
 * When an selection is made from the menu, the corresponding OpMode
 * class is instantiated on the Robot Controller and executed.
 * <p>
 * This particular OpMode just executes a basic Tank Drive Teleop for a two wheeled robot
 * It includes all the skeletal structure that all iterative OpModes contain.
 * <p>
 * Use Android Studios to Copy this Class, and Paste it into your team's code folder with a new name.
 * Remove or comment out the @Disabled line to add this opmode to the Driver Station OpMode list
 */
// Initialize the hardware variables. Note that the strings used here as parameters


@Autonomous(name = "Autonomous", group = "Autonomous")
public class AutonomousTLV extends OpMode {
    // Declare OpMode members.
    private Robot robot;
    private double setPoint;
    private double globalAngle;

    private void Drive(int setPointMeters, double speed){
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
        }


        private boolean Turn(double setPoint){
            double fix = robot.getTurnPID().angle(robot.getLastAngles().firstAngle, setPoint, 1);
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


        private void Foundation(int hold) {
            if (hold == 0) {
                robot.getServoFoundationL().setPosition(0.7);
                robot.getServoFoundationR().setPosition(0.7);
            } else if (hold == 1) {
                robot.getServoFoundationL().setPosition(1);
                robot.getServoFoundationR().setPosition(1);
            }
        }


        private void Lift(int encoder){
            robot.getLiftMotor().setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

            robot.getLiftMotor().setTargetPosition(encoder);

            robot.getLiftMotor().setMode(DcMotor.RunMode.RUN_TO_POSITION);

            robot.getLiftMotor().setPower(0.7);

            while (robot.getLiftMotor().isBusy()){

            }

            robot.getLiftMotor().setPower(0);

            robot.getLiftMotor().setMode(DcMotor.RunMode.RUN_USING_ENCODERS);
        }


        private void Collect(int open){
            if (open == 1){
                robot.getCollectL().setPower(1);
                robot.getCollectR().setPower(1);
                if (robot.getTouchSensorC().isPressed()){
                    robot.getCollectL().setPower(0);
                    robot.getCollectR().setPower(0);
                }else {
                    robot.getCollectL().setPower(1);
                    robot.getCollectR().setPower(1);
                }
            }
            else if (open == 0){
                robot.getCollectL().setPower(0);
                robot.getCollectR().setPower(0);
            }
            else if (open == 2){
                robot.getCollectL().setPower(-1);
                robot.getCollectR().setPower(-1);
            }
        }

    private boolean alpha;
        private boolean Detect(boolean on){
            if (on) {
                if (robot.getColorSensorS().alpha() <= 110) {
                    alpha = true;
                }else{
                    alpha = false;
                }
            }return alpha;
        }

        private void Grab(int take){
            if (take == 1){
                robot.getGrabServo().setPosition(1);
            }else if (take == 0) {
                robot.getGrabServo().setPosition(0);
            }
        }

        private void Hex(int setPointH){
            robot.getHexMotor().setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

            robot.getHexMotor().setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

            robot.getHexMotor().setTargetPosition(setPointH);

            robot.getHexMotor().setMode(DcMotor.RunMode.RUN_TO_POSITION);

            robot.getHexMotor().setPower(0.7);

            if (setPointH >= 0){
                robot.getHexMotor().setPower(0.5);
            }

            while (robot.getHexMotor().isBusy()){

            }

            robot.getHexMotor().setPower(0);

            robot.getHexMotor().setMode(DcMotor.RunMode.RUN_USING_ENCODERS);
        }

        private void LazySusan(int spin){
            if (spin == 1){
                robot.getLazySusan().setPosition(0.45);
            }else if (spin == 0) {
                robot.getLazySusan().setPosition(0.15);
            }
        }

    private boolean RedBridge;
    private boolean BlueBrdge;
        private boolean Line(boolean work){
            if (work){
                while (robot.getColorSensorL().red() <= 9 && robot.getColorSensorL().blue() <= 0){
                    RedBridge = true;
                    BlueBrdge = false;
                }
            }else {
                while (robot.getColorSensorL().red() <= 0 && robot.getColorSensorL().blue() <= 9){
                    RedBridge = false;
                    BlueBrdge = true;
                }
            }return BlueBrdge && RedBridge;
        }




    @Override
    public void init() {
        Drive(500,0.75);
        Turn(90);
        Drive(-250, 0.65);
        Turn(0);
        Foundation(0);
        Foundation(1);
        Lift(500);
        Detect(true);
        Grab(1);
        Grab(0);
        LazySusan(1);
        LazySusan(0);
        Line(true);
        Line(false);


    }

    /*
     * Code to run REPEATEDLY after the driver hits INIT, but before they hit PLAY
     */
    @Override
    public void init_loop() {
    }

    /*
     * Code to run ONCE when the driver hits PLAY
     */
    @Override
    public void start() {
    }

    /*
     * Code to run REPEATEDLY after the driver hits PLAY but before they hit STOP
     */

    @Override
    public void loop() {
    }

    /*
     * Code to run ONCE after the driver hits STOP
     */
    @Override
    public void stop() {
    }
}
