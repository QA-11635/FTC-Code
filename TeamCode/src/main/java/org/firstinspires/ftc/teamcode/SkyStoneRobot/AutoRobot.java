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


@Autonomous(name = "AutoRobot", group = "Autonomous")
public class AutoRobot extends OpMode {
    // Declare OpMode members.

    private Robot robot;

    @Override
    public void init() {
        robot = new Robot(hardwareMap, telemetry);
        robot.runAutonomous(new Action[]{

/* BLUE ALLIANCE: */
                //fOUNDATION SIDE FOUNDATION;

//                new AutoFoundation(0),
//                new AutoDrive(-2800,0.75),
//                new AutoFoundation(0),
//                new AutoFoundation(1),
//                new AutoDrive(0,0),
//                new AutoFoundation(1),
//                new AutoTurn(20),
//                new AutoDrive(1800,0.75),
//                new AutoTurn(90),
//                new AutoDrive(-700,0.75),
//                new AutoFoundation(0),
//                new AutoDrive(4000,0.75)

                //FOUNDATION SIDE BRIDGE;

//                new AutoDrive(3000)


                //SKYSTONE SIDE BRIDGE;

//                new AutoDrive(3000)


                //SKYSTONE SIDE SKYSTONE;

                new AutoLift(1100),
                new AutoCollect(1),
                new AutoDrive(2350,0.75),
                new AutoCollect(0),
                new AutoTurn(-35),
                new AutoCollect(1),
                new AutoDrive(650, 0.6),
                new AutoCollect(1),
                new AutoDrive(-650,0.75),
                new AutoTurn(-90),
                new AutoLift(-1100),
                new AutoGrab(0),
                new AutoDrive(-4700,0.75),
                new AutoTurn(-180),
                new AutoDrive(1000,0.75),
                new AutoFoundation(0),
                new AutoFoundation(1),
                new AutoDrive(0,0),
                new AutoFoundation(1),
                new AutoTurn(20),
                new AutoDrive(1800,0.75),
                new AutoTurn(90),
                new AutoDrive(-700,0.75),
                new AutoFoundation(0),
                new AutoDrive(4000,0.75)

//                new AutoDrive(1000),
//                new AutoDetect(true),
//                new AutoDrive(-500),
//                new AutoDetect(true),
//                new AutoTurn(0),
//                new AutoDrive(250),
//                new AutoTurn(-10),
//                new AutoDrive(700),
//                new AutoDrive(-700),
//                new AutoTurn(-90),
//                new AutoDetect(true),
//                new AutoDrive(1300),
//                new AutoDetect(true),
//                new AutoDrive(-600),
//                new AutoTurn(0),
//                new AutoDrive(250),
//                new AutoTurn(-10),
//                new AutoDrive(700),
//                new AutoDrive(-700),
//                new AutoTurn(-90),
//                new AutoDetect(true),
//                new AutoDrive(1300),
//                new AutoDetect(true),
//                new AutoDrive(-600),
//                new AutoTurn(0),
//                new AutoDrive(250),
//                new AutoTurn(-10),
//                new AutoDrive(700),
//                new AutoDrive(-700),
//                new AutoTurn(-90),
//                new AutoGrab(1),
//                new AutoLift(-1100),
//                new AutoGrab(0),
//                new AutoDrive(-6000),
//                new AutoHex(90),
//                new AutoDrive(3000)










        /*RED ALLIANCE:*/
                //fOUNDATION SIDE FOUNDATION;

//                new AutoFoundation(0),
//                new AutoDrive(-2700,0.75),
//                new AutoDrive(-70, 0.6),
//                new AutoFoundation(0),
//                new AutoFoundation(1),
//                new AutoDrive(0,0),
//                new AutoFoundation(1),
//                new AutoTurn(-15),
//                new AutoDrive(1900,0.75),
//                new AutoTurn(-90),
//                new AutoDrive(-700,0.75),
//                new AutoFoundation(0),
//                new AutoDrive(4000,0.75)

                //FOUNDATION SIDE BRIDGE;

//                new AutoDrive(3000)

                //SKYSTONE SIDE BRIDGE;

//                new AutoDrive(3000)


                //SKYSTONE SIDE SKYSTONE;

//                new AutoLift(-1100),
//                new AutoDrive(2150, 0.75),
//                new AutoTurn(-90),
//                new AutoDrive(-2400,0.75),
//                new AutoTurn(-45),
//                new AutoCollect(1),
//                new AutoDrive(2200,0.75),
//                new AutoDrive(-2400,0.75),
//                new AutoTurn(-90),
//                new AutoCollect(0),
//                new AutoLift(1100),
//                new AutoDrive(3700,0.75)




        });
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
        robot.getScheduler().loop();
    }

    /*
     * Code to run ONCE after the driver hits STOP
     */
    @Override
    public void stop() {
     }
}
