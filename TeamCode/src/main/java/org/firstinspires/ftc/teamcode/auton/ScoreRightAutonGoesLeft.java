package org.firstinspires.ftc.teamcode.auton;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;


@Autonomous(name = "Score right(left)")
public class ScoreRightAutonGoesLeft extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);

        Pose2d startPose = new Pose2d(Cord(2) ,Cord(-3), Math.toRadians(90));

        drive.setPoseEstimate(startPose);

        TrajectorySequence trajSeq = drive.trajectorySequenceBuilder(startPose)
                .forward(60)
                .back(12)

                //first cone
                .lineToLinearHeading(new Pose2d(Cord(1.5) ,Cord(-1), Math.toRadians(90)))
                .forward(5)

                .addTemporalMarker(0.25, () -> {

                })

                .waitSeconds(0.5)

                .back(5)


                //1st stack
                .lineToSplineHeading(new Pose2d(Cord(3.1) ,Cord(-1), Math.toRadians(0)))

                .addTemporalMarker(0.25, () -> {

                })

                .waitSeconds(0.5)
                .lineToSplineHeading(new Pose2d(Cord(1.5) ,Cord(-1), Math.toRadians(90)))
                .forward(5)

                .addTemporalMarker(0.25, () -> {

                })

                .waitSeconds(0.5)
                .back(5)


                //2nd stack
                .lineToSplineHeading(new Pose2d(Cord(3.1) ,Cord(-1), Math.toRadians(0)))

                .addTemporalMarker(0.25, () -> {

                })

                .waitSeconds(0.5)
                .lineToSplineHeading(new Pose2d(Cord(1.5) ,Cord(-1), Math.toRadians(90)))
                .forward(5)
                .back(5)

                .addTemporalMarker(0.25, () -> {

                })

                .waitSeconds(0.5)

                //3rd stack
                .lineToSplineHeading(new Pose2d(Cord(3.1) ,Cord(-1), Math.toRadians(0)))

                .addTemporalMarker(0.25, () -> {

                })

                .waitSeconds(0.5)
                .lineToSplineHeading(new Pose2d(Cord(1.5) ,Cord(-1), Math.toRadians(90)))
                .forward(5)
                .back(5)

                .addTemporalMarker(0.25, () -> {

                })

                .waitSeconds(0.5)




                .build();

        Trajectory park3 = drive.trajectoryBuilder(startPose)
                .lineToSplineHeading(new Pose2d(Cord(3.1) ,Cord(-1), Math.toRadians(0)))
                .build();

        waitForStart();

        if (!isStopRequested())
            drive.followTrajectorySequence(trajSeq);
            drive.followTrajectory(park3);
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