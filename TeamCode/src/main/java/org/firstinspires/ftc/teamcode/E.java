package org.firstinspires.ftc.teamcode;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;


public class E extends LinearOpMode {
    @Override
    public void runOpMode() {

        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);

        //simple trajectory BUt sadly, infite power does not work the fix can be done by calling two separate Trajs or making a Spline Traj.
        Trajectory myTrajectory = drive.trajectoryBuilder(new Pose2d())
                .strafeRight(10)
                .forward(5)
                .build();


        //example of a spline Traj
        Trajectory traj = drive.trajectoryBuilder(new Pose2d())
                //.splineTo(new Vector2d(x1, y1), heading)
                .splineTo(new Vector2d(10,10), Math.toRadians(30))
                .build();


        //a trajectory is 1 continuous movement. If there was to be a stop, a new Traj would have to be started.


        Trajectory trajReverse = drive.trajectoryBuilder(new Pose2d(), true) /* the true shows that it runs the trajectory reversed. True is just 180 degreese turned, which is reverse. U can switch to any value of this using*/ /* Math.toRadians(x) */
                //this code represents traj, and it reverses it.
                .splineTo(new Vector2d(10,10), Math.toRadians(30))
                .build();


        waitForStart();

        if(isStopRequested()) return;

        drive.followTrajectory(myTrajectory);
        drive.followTrajectory(traj);
        drive.followTrajectory(trajReverse);


    }
}