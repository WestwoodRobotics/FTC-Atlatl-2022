package org.firstinspires.ftc.teamcode.Tests;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;


@Autonomous(name = "Score left(right)")
public class ScoreLefttAutonGoesRIght extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);

        Pose2d startPose = new Pose2d(Cord(2) ,Cord(-3), Math.toRadians(90));

        drive.setPoseEstimate(startPose);

        TrajectorySequence trajSeq = drive.trajectorySequenceBuilder(startPose)
                //setup
                .waitSeconds(0.5)
                .forward(54)
                //high
                .back(6)
                .strafeRight(12)
                //preload drop
                .forward(5)
                .waitSeconds(0.5)
                .back(5)
                //go to stack
                .turn(Math.toRadians(90))
                .forward(40)
                .waitSeconds(0.5)
                //high
                .back(40)
                .turn(Math.toRadians(-90))
                //1st drop
                .forward(5)
                .waitSeconds(0.5)
                .back(5)
                //go to stack
                .turn(Math.toRadians(90))
                .forward(40)
                .waitSeconds(0.5)
                //high
                .back(40)
                .turn(Math.toRadians(-90))
                //2nd drop
                .forward(5)
                .waitSeconds(0.5)
                .back(5)
                //left park
                .strafeLeft(36)
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

}