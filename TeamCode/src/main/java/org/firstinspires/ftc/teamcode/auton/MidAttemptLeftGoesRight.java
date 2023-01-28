package org.firstinspires.ftc.teamcode.auton;

/*
 * Copyright (c) 2021 OpenFTC Team
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in all
 * copies or substantial portions of the Software.
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
 * SOFTWARE.
 */

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.openftc.apriltag.AprilTagDetection;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;

import java.util.ArrayList;
import java.util.HashMap;

@Autonomous(name="Cone Auton Left (right)")
public class MidAttemptLeftGoesRight extends LinearOpMode {
    // CHANGE CODE(change string according to what you named your motors)
    String frontLeftM = "leftFront";
    String frontRightM = "rightFront";
    String backLeftM = "leftBack";
    String backRightM = "rightBack";
    // END OG CHANE CODE

    // MCODE
    DcMotorEx frontLeft = null;
    DcMotorEx frontRight = null;
    DcMotorEx backLeft = null;
    DcMotorEx backRight = null;
    DcMotorEx lift = null;
    Servo intake = null;
    Servo aligner = null;
    // END MCODE

    // CHANGE CODE
    // if you have a 1:4 and then a 1:5 then you gear ratio is 1:20
    // in that case just type 20 in the gear ratio variable value
    int gearRatio = 20;
    double wheelRadius = 3.77953 / 2; // value in inches
    double wheelCircumference = 2 * Math.PI * wheelRadius;
    double trackWidth = 13.5; // value in inches
    // go to link to see what track width is
    //https://learnroadrunner.com/assets/img/wes-bot-edit-half.a0bf7846.jpg
    double robotLength = 10.5; // value in inches
    // length is the opposite of width
    // END CHANGE CODE

    // MCODE
    HashMap<Integer, Double> actualGearRatio = new HashMap<>();
    int revMotorTicksPerRotation = 28;
    int ticksPerRotation = revMotorTicksPerRotation * gearRatio;
    double actualTicksPerRotation = 0.0;
    int matLength = 24; // inches
    int turn90DegTicks = 0;
    int forwardTicksForMat = 0; // mat is 24x24 inches
    int forwardTicksForHalfMat = 0;
    // the variable is meant to hold how many ticks to drive forward 24 inches

    // c^2 = a^2 + b^2 -> c = sqrt(a^2 + b^2)
    double robotDiameter = Math.sqrt(Math.pow(trackWidth, 2) + Math.pow(robotLength, 2));
    double robotRadius = robotDiameter / 2.0;
    // C = 2PIr
    double robotCircumference = 2 * Math.PI * robotRadius;
    double robotHalfCircumference = robotCircumference / 2.0;

    // END MCODE

    OpenCvCamera camera;
    AprilTagDetectionPipeline aprilTagDetectionPipeline;

    static final double FEET_PER_METER = 3.28084;

    // Lens intrinsics
    // UNITS ARE PIXELS
    // NOTE: this calibration is for the C920 webcam at 800x448.
    // You will need to do your own calibration for other configurations!
    double fx = 578.272;
    double fy = 578.272;
    double cx = 402.145;
    double cy = 221.506;

    // UNITS ARE METERS
    double tagsize = 0.166;

    //    int ID_TAG_OF_INTEREST = 18; // Tag ID 18 from the 36h11 family
    int LEFT = 3;
    int MIDDLE = 2;
    int RIGHT = 1;

    AprilTagDetection tagOfInterest = null;

    @Override
    public void runOpMode() {
        // MCODE
        actualGearRatio.put(9, 8.4);
        actualGearRatio.put(12, 10.5);
        actualGearRatio.put(15, 15.2);
        actualGearRatio.put(16, 13.1);
        actualGearRatio.put(20, 18.9);
        actualGearRatio.put(25, 27.4);
        // END MCODE

        // MCODE
        actualTicksPerRotation = (double) (revMotorTicksPerRotation * actualGearRatio.get(gearRatio));
        forwardTicksForMat = (int) ((matLength / wheelCircumference) * actualTicksPerRotation);
        forwardTicksForHalfMat = forwardTicksForMat / 2;
        turn90DegTicks = (int) ((robotHalfCircumference / wheelCircumference) * actualTicksPerRotation);
        // END MCODE

        //MCODE
        frontLeft = hardwareMap.get(DcMotorEx.class, frontLeftM);
        frontRight = hardwareMap.get(DcMotorEx.class, frontRightM);
        backLeft = hardwareMap.get(DcMotorEx.class, backLeftM);
        backRight = hardwareMap.get(DcMotorEx.class, backRightM);

        //Lift and intake hardware map
        lift = hardwareMap.get(DcMotorEx.class, "lift");
        lift = hardwareMap.get(DcMotorEx.class, "lift");
        intake = hardwareMap.get(Servo.class, "intake");
        aligner = hardwareMap.get(Servo.class, "aligner");


        // END MCODE

        // CHANGE CODE
        /*
        (sample mecanum equation: drive +/- strafe +/- turn)
        if I set power of 1 on drive variable it should drive forward
        if I set power of 1 on strafe variable it should strafe right
        if I set power of 1 on turn variable it should turn right
         */
        frontLeft.setDirection(DcMotorEx.Direction.REVERSE);
        frontRight.setDirection(DcMotorEx.Direction.FORWARD);
        backLeft.setDirection(DcMotorEx.Direction.REVERSE);
        backRight.setDirection(DcMotorEx.Direction.FORWARD);

        lift.setDirection(DcMotor.Direction.FORWARD);
        // END CHANGE CODE

        // MCODE
        frontLeft.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        frontRight.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        backLeft.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        backRight.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);

        lift.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        frontLeft.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        frontRight.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        backLeft.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        backRight.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);

        lift.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        frontLeft.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
        frontRight.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
        backLeft.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
        backRight.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);

        lift.setTargetPosition(0);
        lift.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        // END MCODE

        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        camera = OpenCvCameraFactory.getInstance().createWebcam(hardwareMap.get(WebcamName.class, "Webcam 1"), cameraMonitorViewId);
        aprilTagDetectionPipeline = new AprilTagDetectionPipeline(tagsize, fx, fy, cx, cy);

        camera.setPipeline(aprilTagDetectionPipeline);
        camera.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener() {
            @Override
            public void onOpened() {
                camera.startStreaming(1280, 720, OpenCvCameraRotation.UPRIGHT);
            }

            @Override
            public void onError(int errorCode) {

            }
        });

        telemetry.setMsTransmissionInterval(50);

        /*
         * The INIT-loop:
         * This REPLACES waitForStart!
         */
        while (!isStarted() && !isStopRequested()) {
            ArrayList<AprilTagDetection> currentDetections = aprilTagDetectionPipeline.getLatestDetections();

            if (currentDetections.size() != 0) {
                boolean tagFound = false;

                for (AprilTagDetection tag : currentDetections) {
                    if (tag.id == LEFT || tag.id == MIDDLE || tag.id == RIGHT) {
                        tagOfInterest = tag;
                        tagFound = true;
                        break;
                    }
                }

                if (tagFound) {
                    telemetry.addLine("Tag of interest is in sight!\n\nLocation data:");
                    tagToTelemetry(tagOfInterest);
                } else {
                    telemetry.addLine("Don't see tag of interest :(");

                    if (tagOfInterest == null) {
                        telemetry.addLine("(The tag has never been seen)");
                    } else {
                        telemetry.addLine("\nBut we HAVE seen the tag before; last seen at:");
                        tagToTelemetry(tagOfInterest);
                    }
                }

            } else {
                telemetry.addLine("Don't see tag of interest :(");

                if (tagOfInterest == null) {
                    telemetry.addLine("(The tag has never been seen)");
                } else {
                    telemetry.addLine("\nBut we HAVE seen the tag before; last seen at:");
                    tagToTelemetry(tagOfInterest);
                }

            }

            telemetry.update();
            sleep(20);
        }

        /*
         * The START command just came in: now work off the latest snapshot acquired
         * during the init loop.
         */

        /* Update the telemetry */
        if (tagOfInterest != null) {
            telemetry.addLine("Tag snapshot:\n");
            tagToTelemetry(tagOfInterest);
            telemetry.update();
        } else {
            telemetry.addLine("No tag snapshot available, it was never sighted during the init loop :(");
            telemetry.update();
        }


        aligner.setPosition(1);
        aligner.setPosition(1);
        aligner.setPosition(1);
        intake.setPosition(0);
        intake.setPosition(0);
        intake.setPosition(0);
        intake.setPosition(0);
        sleep(1000);
        //aligner, claw setup
        {
            intake.setPosition(0);
            intake.setPosition(0);
            intake.setPosition(0);

            lift.setTargetPosition(1500);

            lift.setPower(1);
            sleep(3);

            aligner.setPosition(0);

        }

        // MOVE TO JUNCTION
        {
            frontLeft.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
            frontRight.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
            backLeft.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
            backRight.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);

            frontRight.setTargetPosition(1400);
            backRight.setTargetPosition(1400);
            frontLeft.setTargetPosition(1800);
            backLeft.setTargetPosition(1800);
            lift.setTargetPosition(3000);

            frontLeft.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);
            frontRight.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);
            backLeft.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);
            backRight.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);

            frontLeft.setPower(0.5);
            frontRight.setPower(0.5*10/15);
            backLeft.setPower(0.5);
            backRight.setPower(0.5*10/15);
            lift.setPower(1);


            sleep(6000);

            frontLeft.setPower(0);
            frontRight.setPower(0);
            backLeft.setPower(0);
            backRight.setPower(0);
        }

        //drop cone
        {
            lift.setTargetPosition(2000);
            lift.setPower(-1);
            sleep(1);
            intake.setPosition(0.6);
        }

        //raise aligner
        {
            frontLeft.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
            backLeft.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
            frontRight.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
            backRight.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);


            frontLeft.setTargetPosition(-150);
            frontRight.setTargetPosition(-150);
            backLeft.setTargetPosition(-150);
            backRight.setTargetPosition(-150);
            lift.setTargetPosition(1750);

            frontLeft.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);
            frontRight.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);
            backLeft.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);
            backRight.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);

            frontLeft.setPower(0.5);
            frontRight.setPower(.5);
            backLeft.setPower(0.5);
            backRight.setPower(.5);
            lift.setPower(-1);

            sleep(1000);

            frontLeft.setPower(0);
            frontRight.setPower(0);
            backLeft.setPower(0);
            backRight.setPower(0);
            lift.setPower(-1);
            aligner.setPosition(1);
            intake.setPosition(0);
        }


        frontLeft.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        backLeft.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        frontRight.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        backRight.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);

        frontLeft.setTargetPosition(-220);
        backLeft.setTargetPosition(-220);
        frontRight.setTargetPosition(220);
        backRight.setTargetPosition(220);
        lift.setTargetPosition(0);

        frontLeft.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);
        backLeft.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);
        frontRight.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);
        backRight.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);

        frontLeft.setPower(.3);
        frontRight.setPower(.3);
        backLeft.setPower(.3);
        backRight.setPower(.3);
        lift.setPower(-1);

        sleep(1000);

        frontLeft.setPower(0);
        frontRight.setPower(0);
        backLeft.setPower(0);
        backRight.setPower(0);


        frontLeft.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        backLeft.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        frontRight.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        backRight.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);

        frontLeft.setTargetPosition(-180);
        backLeft.setTargetPosition(-180);
        frontRight.setTargetPosition(-180);
        backRight.setTargetPosition(-180);
        lift.setTargetPosition(0);

        frontLeft.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);
        backLeft.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);
        frontRight.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);
        backRight.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);

        frontLeft.setPower(.3);
        frontRight.setPower(.3);
        backLeft.setPower(.3);
        backRight.setPower(.3);
        lift.setPower(-1);

        sleep(1000);

        frontLeft.setPower(0);
        frontRight.setPower(0);
        backLeft.setPower(0);
        backRight.setPower(0);

        if (tagOfInterest == null || tagOfInterest.id == LEFT) {
            frontLeft.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
            frontRight.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
            backLeft.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
            backRight.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);

            frontLeft.setTargetPosition(-500);
            frontRight.setTargetPosition(500);
            backLeft.setTargetPosition(500);
            backRight.setTargetPosition(-500);

            frontLeft.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);
            frontRight.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);
            backLeft.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);
            backRight.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);

            frontLeft.setPower(-.3);
            frontRight.setPower(.3);
            backLeft.setPower(.3);
            backRight.setPower(-.3);
            // END MCODE

            // CHANGE CODE
            // change value according to how long it takes robot to reach wanted position
            sleep(5000); // 5 seconds
            // END CHANGE CODE

            // MCODE
            frontLeft.setPower(0);
            frontRight.setPower(0);
            backLeft.setPower(0);
            backRight.setPower(0);
            // END MCODE


        } else if ( tagOfInterest.id == MIDDLE) {
            frontLeft.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
            frontRight.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
            backLeft.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
            backRight.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);

            frontLeft.setTargetPosition(690);
            frontRight.setTargetPosition(-690);
            backLeft.setTargetPosition(-690);
            backRight.setTargetPosition(690);

            frontLeft.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);
            frontRight.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);
            backLeft.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);
            backRight.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);

            frontLeft.setPower(-.3);
            frontRight.setPower(.3);
            backLeft.setPower(.3);
            backRight.setPower(-.3);
            // END MCODE

            // CHANGE CODE
            // change value according to how long it takes robot to reach wanted position
            sleep(5000); // 5 seconds
            // END CHANGE CODE

            // MCODE
            frontLeft.setPower(0);
            frontRight.setPower(0);
            backLeft.setPower(0);
            backRight.setPower(0);
            // END MCODE

            // MOVE FORWARD 1.5 MAT LENGTH (END)

        } else {
            frontLeft.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
            frontRight.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
            backLeft.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
            backRight.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);

            frontLeft.setTargetPosition(-690);
            frontRight.setTargetPosition(690);
            backLeft.setTargetPosition(690);
            backRight.setTargetPosition(-690);

            frontLeft.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);
            frontRight.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);
            backLeft.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);
            backRight.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);

            frontLeft.setPower(-.3);
            frontRight.setPower(.3);
            backLeft.setPower(.3);
            backRight.setPower(-.3);
            // END MCODE

            // CHANGE CODE
            // change value according to how long it takes robot to reach wanted position
            sleep(5000); // 5 seconds
            // END CHANGE CODE

            // MCODE
            frontLeft.setPower(0);
            frontRight.setPower(0);
            backLeft.setPower(0);
            backRight.setPower(0);
            // END MCODE


        }


        telemetry.update();
        /* You wouldn't have this in your autonomous, this is just to prevent the sample from ending */
        while (opModeIsActive()) {sleep(20);}
    }

    void tagToTelemetry(AprilTagDetection detection)
    {
        telemetry.addLine(String.format("\nDetected tag ID=%d", detection.id));
        telemetry.addLine(String.format("Translation X: %.2f feet", detection.pose.x*FEET_PER_METER));
        telemetry.addLine(String.format("Translation Y: %.2f feet", detection.pose.y*FEET_PER_METER));
        telemetry.addLine(String.format("Translation Z: %.2f feet", detection.pose.z*FEET_PER_METER));
        telemetry.addLine(String.format("Rotation Yaw: %.2f degrees", Math.toDegrees(detection.pose.yaw)));
        telemetry.addLine(String.format("Rotation Pitch: %.2f degrees", Math.toDegrees(detection.pose.pitch)));
        telemetry.addLine(String.format("Rotation Roll: %.2f degrees", Math.toDegrees(detection.pose.roll)));
    }
}