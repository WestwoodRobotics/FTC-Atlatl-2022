package org.firstinspires.ftc.teamcode.Tests;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;


@Autonomous(name = "rightTest")
public class ScoreRightAuton extends LinearOpMode {

    public boolean clawOpen = true;
    public Servo claw1 = null;
    public Servo claw2 = null;

    public DcMotorEx lift = null;


    @Override
    public void runOpMode() throws InterruptedException {
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

        drive.setPoseEstimate(startPose);

        TrajectorySequence trajSeq = drive.trajectorySequenceBuilder(startPose)
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
                .strafeLeft(12)
                //preload drop
                .forward(5)
                .addTemporalMarker(0, () -> {
                    liftMove(2000);
                    clawToggle();
                })
                .waitSeconds(0.5)
                .back(5)
                //go to stack
                .turn(Math.toRadians(-90))

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
                .turn(Math.toRadians(90))
                //1st drop
                .forward(5)
                .addTemporalMarker(0, () -> {
                    liftMove(2000);
                    clawToggle();
                })
                .waitSeconds(0.5)
                .back(5)
                //go to stack
                .turn(Math.toRadians(-90))
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
                .turn(Math.toRadians(90))
                //2nd drop
                .forward(5)
                .addTemporalMarker(0, () -> {
                    liftMove(0);
                    clawToggle();
                })
                .waitSeconds(0.5)
                .back(5)
                //left park
                .strafeRight(40)

                .build();


        waitForStart();

        if (!isStopRequested())
            drive.followTrajectorySequence(trajSeq);
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