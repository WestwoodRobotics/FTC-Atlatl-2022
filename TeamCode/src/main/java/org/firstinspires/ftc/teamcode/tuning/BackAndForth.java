package org.firstinspires.ftc.teamcode.tuning;

import com.acmerobotics.roadrunner.PosePath;
import com.acmerobotics.roadrunner.Transform2;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.MecanumDrive;

public final class BackAndForth extends LinearOpMode {

    public static double DISTANCE = 50;

    @Override
    public void runOpMode() throws InterruptedException {
        MecanumDrive drive = new MecanumDrive(hardwareMap);

        PosePath forward = drive.pathBuilder(new Transform2(0, 0, 0), 0)
                .forward(DISTANCE)
                .build();

        PosePath backward = drive.pathBuilder(forward.end(1).value(), 0)
                .forward(-DISTANCE)
                .build();

        waitForStart();

        while (opModeIsActive()) {
            drive.followPath(forward).runBlocking();
            drive.followPath(backward).runBlocking();
        }
    }
}
