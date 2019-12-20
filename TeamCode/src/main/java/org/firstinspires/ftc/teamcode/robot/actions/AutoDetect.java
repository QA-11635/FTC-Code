package org.firstinspires.ftc.teamcode.robot.actions;
//
//import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
//import com.qualcomm.robotcore.eventloop.opmode.Disabled;
//import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
//import com.qualcomm.robotcore.util.ElapsedTime;
//
//import org.opencv.core.Core;
//import org.opencv.core.Mat;
//import org.opencv.core.MatOfPoint;
//import org.opencv.core.Point;
//import org.opencv.core.Scalar;
//import org.opencv.imgproc.Imgproc;
//import org.openftc.easyopencv.OpenCvCamera;
//import org.openftc.easyopencv.OpenCvCameraRotation;
//import org.openftc.easyopencv.OpenCvInternalCamera;
//import org.openftc.easyopencv.OpenCvPipeline;
//
//import java.util.ArrayList;
//import java.util.List;
//
//
///**
// * Created by maryjaneb  on 11/13/2016.
// *
// * nerverest ticks
// * 60 1680
// * 40 1120
// * 20 560
// *
// * monitor: 640 x 480
// *YES
// */
//@Autonomous(name= "opencvSkystoneDetector", group="Sky autonomous")
//@Disabled//comment out this line before using
//public class AutoDetect extends LinearOpMode {
//    private ElapsedTime runtime = new ElapsedTime();
//
//    //0 means skystone, 1 means yellow stone
//    //-1 for debug, but we can keep it like this because if it works, it should change to either 0 or 255
//    private static int valMid = -1;
//    private static int valLeft = -1;
//    private static int valRight = -1;
//
//    private static float rectHeight = .6f/8f;
//    private static float rectWidth = 1.5f/8f;
//
//    private static float offsetX = 0f/8f;//changing this moves the three rects and the three circles left or right, range : (-2, 2) not inclusive
//    private static float offsetY = 0f/8f;//changing this moves the three rects and circles up or down, range: (-4, 4) not inclusive
//
//    private static float[] midPos = {4f/8f+offsetX, 4f/8f+offsetY};//0 = col, 1 = row
//    private static float[] leftPos = {2f/8f+offsetX, 4f/8f+offsetY};
//    private static float[] rightPos = {6f/8f+offsetX, 4f/8f+offsetY};
//    //moves all rectangles right or left by amount. units are in ratio to monitor
//
//    private final int rows = 640;
//    private final int cols = 480;
//
//    OpenCvCamera phoneCam;
//
//    @Override
//    public void runOpMode() throws InterruptedException {
//
//
//        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
//        phoneCam = new OpenCvInternalCamera(OpenCvInternalCamera.CameraDirection.BACK, cameraMonitorViewId);
//        phoneCam.openCameraDevice();//open camera
//        phoneCam.setPipeline(new StageSwitchingPipeline());//different stages
//        phoneCam.startStreaming(rows, cols, OpenCvCameraRotation.UPRIGHT);//display on RC
//        //width, height
//        //width = height in this case, because camera is in portrait mode.
//
//        waitForStart();
//        runtime.reset();
//        while (opModeIsActive()) {
//            telemetry.addData("Values", valLeft+"   "+valMid+"   "+valRight);
//            telemetry.addData("Height", rows);
//            telemetry.addData("Width", cols);
//
//            telemetry.update();
//            sleep(100);
//            //call movement functions
////            strafe(0.4, 200);
////            moveDistance(0.4, 700);
//
//        }
//    }
//
//    //detection pipeline
//    static class StageSwitchingPipeline extends OpenCvPipeline
//    {
//        Mat yCbCrChan2Mat = new Mat();
//        Mat thresholdMat = new Mat();
//        Mat all = new Mat();
//        List<MatOfPoint> contoursList = new ArrayList<>();
//
//        enum Stage
//        {//color difference. greyscale
//            detection,//includes outlines
//            THRESHOLD,//b&w
//            RAW_IMAGE,//displays raw view
//        }
//
//        private Stage stageToRenderToViewport = Stage.detection;
//        private Stage[] stages = Stage.values();
//
//        @Override
//        public void onViewportTapped()
//        {
//            /*
//             * Note that this method is invoked from the UI thread
//             * so whatever we do here, we must do quickly.
//             */
//
//            int currentStageNum = stageToRenderToViewport.ordinal();
//
//            int nextStageNum = currentStageNum + 1;
//
//            if(nextStageNum >= stages.length)
//            {
//                nextStageNum = 0;
//            }
//
//            stageToRenderToViewport = stages[nextStageNum];
//        }
//
//        @Override
//        public Mat processFrame(Mat input)
//        {
//            contoursList.clear();
//            /*
//             * This pipeline finds the contours of yellow blobs such as the Gold Mineral
//             * from the Rover Ruckus game.
//             */
//
//            //color diff cb.
//            //lower cb = more blue = skystone = white
//            //higher cb = less blue = yellow stone = grey
//            Imgproc.cvtColor(input, yCbCrChan2Mat, Imgproc.COLOR_RGB2YCrCb);//converts rgb to ycrcb
//            Core.extractChannel(yCbCrChan2Mat, yCbCrChan2Mat, 2);//takes cb difference and stores
//
//            //b&w
//            Imgproc.threshold(yCbCrChan2Mat, thresholdMat, 102, 255, Imgproc.THRESH_BINARY_INV);
//
//            //outline/contour
//            Imgproc.findContours(thresholdMat, contoursList, new Mat(), Imgproc.RETR_LIST, Imgproc.CHAIN_APPROX_SIMPLE);
//            yCbCrChan2Mat.copyTo(all);//copies mat object
//            //Imgproc.drawContours(all, contoursList, -1, new Scalar(255, 0, 0), 3, 8);//draws blue contours
//
//
//            //get values from frame
//            double[] pixMid = thresholdMat.get((int)(input.rows()* midPos[1]), (int)(input.cols()* midPos[0]));//gets value at circle
//            valMid = (int)pixMid[0];
//
//            double[] pixLeft = thresholdMat.get((int)(input.rows()* leftPos[1]), (int)(input.cols()* leftPos[0]));//gets value at circle
//            valLeft = (int)pixLeft[0];
//
//            double[] pixRight = thresholdMat.get((int)(input.rows()* rightPos[1]), (int)(input.cols()* rightPos[0]));//gets value at circle
//            valRight = (int)pixRight[0];
//
//            //create three points
//            Point pointMid = new Point((int)(input.cols()* midPos[0]), (int)(input.rows()* midPos[1]));
//            Point pointLeft = new Point((int)(input.cols()* leftPos[0]), (int)(input.rows()* leftPos[1]));
//            Point pointRight = new Point((int)(input.cols()* rightPos[0]), (int)(input.rows()* rightPos[1]));
//
//            //draw circles on those points
//            Imgproc.circle(all, pointMid,5, new Scalar( 255, 0, 0 ),1 );//draws circle
//            Imgproc.circle(all, pointLeft,5, new Scalar( 255, 0, 0 ),1 );//draws circle
//            Imgproc.circle(all, pointRight,5, new Scalar( 255, 0, 0 ),1 );//draws circle
//
//            //draw 3 rectangles
//            Imgproc.rectangle(//1-3
//                    all,
//                    new Point(
//                            input.cols()*(leftPos[0]-rectWidth/2),
//                            input.rows()*(leftPos[1]-rectHeight/2)),
//                    new Point(
//                            input.cols()*(leftPos[0]+rectWidth/2),
//                            input.rows()*(leftPos[1]+rectHeight/2)),
//                    new Scalar(0, 255, 0), 3);
//            Imgproc.rectangle(//3-5
//                    all,
//                    new Point(
//                            input.cols()*(midPos[0]-rectWidth/2),
//                            input.rows()*(midPos[1]-rectHeight/2)),
//                    new Point(
//                            input.cols()*(midPos[0]+rectWidth/2),
//                            input.rows()*(midPos[1]+rectHeight/2)),
//                    new Scalar(0, 255, 0), 3);
//            Imgproc.rectangle(//5-7
//                    all,
//                    new Point(
//                            input.cols()*(rightPos[0]-rectWidth/2),
//                            input.rows()*(rightPos[1]-rectHeight/2)),
//                    new Point(
//                            input.cols()*(rightPos[0]+rectWidth/2),
//                            input.rows()*(rightPos[1]+rectHeight/2)),
//                    new Scalar(0, 255, 0), 3);
//
//            switch (stageToRenderToViewport)
//            {
//                case THRESHOLD:
//                {
//                    return thresholdMat;
//                }
//
//                case detection:
//                {
//                    return all;
//                }
//
//                case RAW_IMAGE:
//                {
//                    return input;
//                }
//
//                default:
//                {
//                    return input;
//                }
//            }
//        }
//
//    }
//}

/* Copyright (c) 2019 FIRST. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted (subject to the limitations in the disclaimer below) provided that
 * the following conditions are met:
 *
 * Redistributions of source code must retain the above copyright notice, this list
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

//package org.firstinspires.ftc.robotcontroller.external.samples;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import java.util.List;

import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer.CameraDirection;
import org.firstinspires.ftc.robotcore.external.tfod.TFObjectDetector;
import org.firstinspires.ftc.robotcore.external.tfod.Recognition;

/**
 * This 2019-2020 OpMode illustrates the basics of using the TensorFlow Object Detection API to
 * determine the position of the Skystone game elements.
 * <p>
 * Use Android Studio to Copy this Class, and Paste it into your team's code folder with a new name.
 * Remove or comment out the @Disabled line to add this opmode to the Driver Station OpMode list.
 * <p>
 * IMPORTANT: In order to use this OpMode, you need to obtain your own Vuforia license key as
 * is explained below.
 */
@TeleOp(name = "Concept: TensorFlow Object Detection", group = "Concept")
@Disabled
public class AutoDetect extends LinearOpMode {
    private static final String TFOD_MODEL_ASSET = "Skystone.tflite";
    private static final String LABEL_FIRST_ELEMENT = "Stone";
    private static final String LABEL_SECOND_ELEMENT = "Skystone";

    /*
     * IMPORTANT: You need to obtain your own license key to use Vuforia. The string below with which
     * 'parameters.vuforiaLicenseKey' is initialized is for illustration only, and will not function.
     * A Vuforia 'Development' license key, can be obtained free of charge from the Vuforia developer
     * web site at https://developer.vuforia.com/license-manager.
     *
     * Vuforia license keys are always 380 characters long, and look as if they contain mostly
     * random data. As an example, here is a example of a fragment of a valid key:
     *      ... yIgIzTqZ4mWjk9wd3cZO9T1axEqzuhxoGlfOOI2dRzKS4T0hQ8kT ...
     * Once you've obtained a license key, copy the string from the Vuforia web site
     * and paste it in to your code on the next line, between the double quotes.
     */
    private static final String VUFORIA_KEY =
            "AeNJe/D/////AAABma3zpeBqhUccl7pXtQuCkBhDy8UlUV9/L3rf6F5bdEBfUEunLrchGEFvQcZtw0IdjOgQR4Tjq47JbgzYMX0s9Nt9a/V3eWDZ3KdpEHP4+MBjy+3f96l1z89VIL8UCbfaC1vgnwb5TRrWESErL2KHLJ0sz24w4iSKjKd+gHsAcQbzhOIBeiNeGxR6/M/E8aUyeoQ1AdnyOQLwsbMEnPrJXkLsY+2+jV0Xj1xqWTC2jPMN13ryFqBk/dkq0z/sEgD0DO3ldBObc3ay36a9nEbtvaVN1pPX2YwKVQLHkjsK/Ymgb7RSK5bI1hpIWOu4swFUVOlrkA7cqUEWDdM4U48lXejr1YMwAj9FnZzz/xjxnata";

    /**
     * {@link #vuforia} is the variable we will use to store our instance of the Vuforia
     * localization engine.
     */
    private VuforiaLocalizer vuforia;

    /**
     * {@link #tfod} is the variable we will use to store our instance of the TensorFlow Object
     * Detection engine.
     */
    private TFObjectDetector tfod;

    @Override
    public void runOpMode() {
        // The TFObjectDetector uses the camera frames from the VuforiaLocalizer, so we create that
        // first.
        initVuforia();

        if (ClassFactory.getInstance().canCreateTFObjectDetector()) {
            initTfod();
        } else {
            telemetry.addData("Sorry!", "This device is not compatible with TFOD");
        }

        /**
         * Activate TensorFlow Object Detection before we wait for the start command.
         * Do it here so that the Camera Stream window will have the TensorFlow annotations visible.
         **/
        if (tfod != null) {
            tfod.activate();
        }

        /** Wait for the game to begin */
        telemetry.addData(">", "Press Play to start op mode");
        telemetry.update();
        waitForStart();

        if (opModeIsActive()) {
            while (opModeIsActive()) {
                if (tfod != null) {
                    // getUpdatedRecognitions() will return null if no new information is available since
                    // the last time that call was made.
                    List<Recognition> updatedRecognitions = tfod.getUpdatedRecognitions();
                    if (updatedRecognitions != null) {
                        telemetry.addData("# Object Detected", updatedRecognitions.size());

                        // step through the list of recognitions and display boundary info.
                        int i = 0;
                        for (Recognition recognition : updatedRecognitions) {
                            telemetry.addData(String.format("label (%d)", i), recognition.getLabel());
                            telemetry.addData(String.format("  left,top (%d)", i), "%.03f , %.03f",
                                    recognition.getLeft(), recognition.getTop());
                            telemetry.addData(String.format("  right,bottom (%d)", i), "%.03f , %.03f",
                                    recognition.getRight(), recognition.getBottom());
                        }
                        telemetry.update();
                    }
                }
            }
        }

        if (tfod != null) {
            tfod.shutdown();
        }
    }

    /**
     * Initialize the Vuforia localization engine.
     */
    private void initVuforia() {
        /*
         * Configure Vuforia by creating a Parameter object, and passing it to the Vuforia engine.
         */
        VuforiaLocalizer.Parameters parameters = new VuforiaLocalizer.Parameters();

        parameters.vuforiaLicenseKey = VUFORIA_KEY;
        parameters.cameraDirection = CameraDirection.BACK;

        //  Instantiate the Vuforia engine
        vuforia = ClassFactory.getInstance().createVuforia(parameters);

        // Loading trackables is not necessary for the TensorFlow Object Detection engine.
    }

    /**
     * Initialize the TensorFlow Object Detection engine.
     */
    private void initTfod() {
        int tfodMonitorViewId = hardwareMap.appContext.getResources().getIdentifier(
                "tfodMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        TFObjectDetector.Parameters tfodParameters = new TFObjectDetector.Parameters(tfodMonitorViewId);
        tfodParameters.minimumConfidence = 0.8;
        tfod = ClassFactory.getInstance().createTFObjectDetector(tfodParameters, vuforia);
        tfod.loadModelFromAsset(TFOD_MODEL_ASSET, LABEL_FIRST_ELEMENT, LABEL_SECOND_ELEMENT);
    }
}
