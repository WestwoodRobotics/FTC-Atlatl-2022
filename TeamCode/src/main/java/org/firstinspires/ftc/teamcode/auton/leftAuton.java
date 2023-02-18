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

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;
import org.openftc.apriltag.AprilTagDetection;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;

import java.util.ArrayList;
import java.util.HashMap;

@Autonomous(name="left auton")
public class leftAuton extends LinearOpMode
{
    // CHANGE CODE(change string according to what you named your motors)
    String frontLeftM = "leftFront";
    String frontRightM = "rightFront";
    String backLeftM = "leftBack";
    String backRightM = "rightBack";
    // END OG CHANE CODE

    public boolean clawOpen = true;
    public Servo claw1 = null;
    public Servo claw2 = null;

    public DcMotorEx lift = null;

    // MCODE
    DcMotorEx frontLeft = null;
    DcMotorEx frontRight = null;
    DcMotorEx backLeft = null;
    DcMotorEx backRight = null;
    // END MCODE

    // CHANGE CODE
    // if you have a 1:4 and then a 1:5 then you gear ratio is 1:20
    // in that case just type 20 in the gear ratio variable value
    int gearRatio = 19;
    double wheelRadius = 3.77953/2; // value in inches
    double wheelCircumference = 2*Math.PI*wheelRadius;
    double trackWidth = 13.5; // value in inches
    // go to link to see what track width is
    //https://learnroadrunner.com/assets/img/wes-bot-edit-half.a0bf7846.jpg
    double robotLength = 10.5; // value in inches
    // length is the opposite of width
    // END CHANGE CODE

    // MCODE
    HashMap<Integer, Double> actualGearRatio = new HashMap<>();
    int revMotorTicksPerRotation = 28;
    int ticksPerRotation = revMotorTicksPerRotation*gearRatio;
    double actualTicksPerRotation = 0.0;
    int matLength = 24; // inch-es
    int turn90DegTicks = 0;
    int forwardTicksForMat = 0; // mat is 24x24 inches
    int forwardTicksForHalfMat = 0;
    // the variable is meant to hold how many ticks to drive forward 24 inches

    // c^2 = a^2 + b^2 -> c = sqrt(a^2 + b^2)
    double robotDiameter = Math.sqrt(Math.pow(trackWidth,2)+Math.pow(robotLength,2));
    double robotRadius = robotDiameter/2.0;
    // C = 2PIr
    double robotCircumference = 2*Math.PI*robotRadius;
    double robotHalfCircumference = robotCircumference/2.0;

    // END MCODE

    OpenCvCamera camera;
    TagDetection aprilTagDetectionPipeline;

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
    int LEFT = 1;
    int MIDDLE = 2;
    int RIGHT = 3;

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
        // END MCODE

        // CHANGE CODE
        /*
        (sample mecanum equation: drive +/- strafe +/- turn)
        if I set power of 1 on drive variable it should drive forward
        if I set power of 1 on strafe variable it should strafe right
        if I set power of 1 on turn variable it should turn right
         */
        frontLeft.setDirection(DcMotor.Direction.REVERSE);
        frontRight.setDirection(DcMotor.Direction.FORWARD);
        backLeft.setDirection(DcMotor.Direction.REVERSE);
        backRight.setDirection(DcMotor.Direction.FORWARD);
        // END CHANGE CODE

        // MCODE
        frontLeft.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        frontRight.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        backLeft.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        backRight.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);

        frontLeft.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        frontRight.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        backLeft.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        backRight.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);

        frontLeft.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
        frontRight.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
        backLeft.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
        backRight.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
        // END MCODE


        claw1 = hardwareMap.get(Servo.class, "claw1");
        claw2 = hardwareMap.get(Servo.class, "claw2");
        lift = hardwareMap.get(DcMotorEx.class, "lift");

        lift.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        lift.setTargetPosition(0);
        lift.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);

        lift.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);

        lift.setDirection(DcMotorEx.Direction.REVERSE);


        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        camera = OpenCvCameraFactory.getInstance().createWebcam(hardwareMap.get(WebcamName.class, "Webcam 1"), cameraMonitorViewId);
        aprilTagDetectionPipeline = new TagDetection(tagsize, fx, fy, cx, cy);

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

        //trajectories
        SampleMecanumDrive drive;
        TrajectorySequence scoring;
        {
            drive = new SampleMecanumDrive(hardwareMap);

            Pose2d startPose = new Pose2d(0, 0, Math.toRadians(90));

            drive.setPoseEstimate(startPose);

            scoring = drive.trajectorySequenceBuilder(startPose)
                    //setup
                    .addTemporalMarker(0, () -> {
                        claw1.setPosition(0.02);
                        claw2.setPosition(0.98);
                        claw1.setPosition(0.02);
                        claw2.setPosition(0.98);
                        claw1.setPosition(0.02);
                        claw2.setPosition(0.98);
                        clawOpen = false;
                    })
                    .forward(64)
                    .addTemporalMarker(1, () -> {
                        liftMove(2750);
                    })
                    .back(11)
                    //high
                    .strafeRight(12)
                    //preload drop
                    .forward(5)
                    .addTemporalMarker(0, () -> {
                        liftMove(2000);
                        clawToggle();
                    })
                    .waitSeconds(0.5)
                    .back(5)
                    //go to stack
                    .turn(Math.toRadians(90))

                    .addTemporalMarker(0, () -> {
                        liftMove(800);
                    })

                    .forward(40)
                    .addTemporalMarker(0, () -> {
                        clawToggle();
                    })
                    .waitSeconds(0.5)
                    //high
                    .addTemporalMarker(0, () -> {
                        liftMove(2750);
                    })
                    .back(40)
                    .turn(Math.toRadians(-90))
                    //1st drop
                    .forward(5)
                    .addTemporalMarker(0, () -> {
                        liftMove(2000);
                        clawToggle();
                    })
                    .waitSeconds(0.5)
                    .back(5)
                    //go to stack
                    .turn(Math.toRadians(90))
                    .addTemporalMarker(0, () -> {
                        liftMove(750);
                    })
                    .forward(40)
                    .addTemporalMarker(0, () -> {
                        clawToggle();
                    })
                    .waitSeconds(0.5)
                    //high
                    .addTemporalMarker(0, () -> {
                        liftMove(2750);
                    })
                    .back(40)
                    .turn(Math.toRadians(-90))
                    //2nd drop
                    .forward(5)
                    .addTemporalMarker(0, () -> {
                        liftMove(0);
                        clawToggle();
                    })
                    .waitSeconds(0.5)
                    .back(5)

                    .build();


        }


        drive.followTrajectorySequence(scoring);
        if (tagOfInterest == null || tagOfInterest.id == LEFT) {
            //left park
            Trajectory park1 = drive.trajectoryBuilder(new Pose2d(0, 0, Math.toRadians(90)))
                    .addTemporalMarker(0, () -> {
                        liftMove(0);
                        clawToggle();
                    })
                    .strafeLeft(40)
                    .build();

            drive.followTrajectory(park1);
        } else if (tagOfInterest.id == MIDDLE) {
            //middle park
            Trajectory park2 = drive.trajectoryBuilder(new Pose2d(0, 0, Math.toRadians(90)))
                    .addTemporalMarker(0, () -> {
                        liftMove(0);
                        clawToggle();
                    })
                    .strafeLeft(12)
                    .build();

            drive.followTrajectory(park2);
        } else {
            //right park
            Trajectory park3 = drive.trajectoryBuilder(new Pose2d(0, 0, Math.toRadians(90)))
                    .addTemporalMarker(0, () -> {
                        liftMove(0);
                        clawToggle();
                    })
                    .strafeLeft(0.001)
                    .build();

            drive.followTrajectory(park3);
        }


        telemetry.update();
        /* You wouldn't have this in your autonomous, this is just to prevent the sample from ending */
        while (opModeIsActive()) {
            sleep(20);
        }
    }

    void tagToTelemetry(AprilTagDetection detection)
    {
        telemetry.addLine(String.format("\nDetected tag ID=%d", detection.id));
    }

    public void clawToggle(){
        if (!clawOpen){
            claw1.setPosition(0.57);
            claw2.setPosition(0.38);
        }else{
            claw1.setPosition(0.02);
            claw2.setPosition(0.98);
        }
        clawOpen = !clawOpen;
    }

    public void liftMove(int pos){
        lift.setTargetPosition(pos);
        lift.setPower(1);
    }
}