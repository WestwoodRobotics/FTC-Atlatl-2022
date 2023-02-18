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
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
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

@Autonomous(name="parking auton")
public class leftAuton extends LinearOpMode
{

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

    public boolean clawOpen = true;
    public Servo claw1 = null;
    public Servo claw2 = null;

    public DcMotorEx lift = null;

    @Override
    public void runOpMode() {

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

        claw1 = hardwareMap.get(Servo.class, "claw1");
        claw2 = hardwareMap.get(Servo.class, "claw2");
        lift = hardwareMap.get(DcMotorEx.class, "lift");

        lift.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        lift.setTargetPosition(0);
        lift.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);

        lift.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);

        lift.setDirection(DcMotorEx.Direction.REVERSE);


        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);

        Pose2d startPose = new Pose2d(Cord(2) ,Cord(-3), Math.toRadians(90));

        //scoring trajectory

        TrajectorySequence park1 = drive.trajectorySequenceBuilder(startPose)
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
                //left park
                .strafeLeft(40)

                .build();

        TrajectorySequence park2 = drive.trajectorySequenceBuilder(startPose)
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
                //left park
                .strafeLeft(12)

                .build();

        TrajectorySequence park3 = drive.trajectorySequenceBuilder(startPose)
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
                //left park
                .build();







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


        if (tagOfInterest == null || tagOfInterest.id == LEFT) {
            drive.followTrajectorySequence(park1);
        } else if (tagOfInterest.id == MIDDLE) {
            drive.followTrajectorySequence(park2);
        } else {
            drive.followTrajectorySequence(park3);
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



        //delete the lines below and see what happens
        telemetry.addLine(String.format("Translation X: %.2f feet", detection.pose.x*FEET_PER_METER));
        telemetry.addLine(String.format("Translation Y: %.2f feet", detection.pose.y*FEET_PER_METER));
        telemetry.addLine(String.format("Translation Z: %.2f feet", detection.pose.z*FEET_PER_METER));
        telemetry.addLine(String.format("Rotation Yaw: %.2f degrees", Math.toDegrees(detection.pose.yaw)));
        telemetry.addLine(String.format("Rotation Pitch: %.2f degrees", Math.toDegrees(detection.pose.pitch)));
        telemetry.addLine(String.format("Rotation Roll: %.2f degrees", Math.toDegrees(detection.pose.roll)));
    }

    public double Cord(double mat){
        if (mat > 0){
            mat = (mat * 24) - 12;
        }else if (mat < 0){
            mat = (mat * 24) + 12;
        }else{
            mat = 0;
        }

        return mat;
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