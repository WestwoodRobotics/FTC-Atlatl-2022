package org.firstinspires.ftc.teamcode.tuning;

import com.acmerobotics.roadrunner.PosePath;
import com.acmerobotics.roadrunner.Position2;
import com.acmerobotics.roadrunner.Transform2;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.MecanumDrive;
import org.firstinspires.ftc.teamcode.TankDrive;

public final class BackAndForth extends LinearOpMode {

    public static double DISTANCE = 50;

    @Override
    public void runOpMode() throws InterruptedException {
        if (TuningOpModes.DRIVE_CLASS.equals(MecanumDrive.class)) {
            MecanumDrive drive = new MecanumDrive(hardwareMap);

            PosePath forward = drive.pathBuilder(new Transform2(0, 0, 0), 0)
                    .forward(DISTANCE)
                    .build();

            PosePath backward = drive.pathBuilder(forward, Math.PI)
                    .forward(-DISTANCE)
                    .build();

            waitForStart();

            while (opModeIsActive()) {
                drive.followPath(forward).runBlocking();
                drive.followPath(backward).runBlocking();
            }
        } else if (TuningOpModes.DRIVE_CLASS.equals(TankDrive.class)) {
            TankDrive drive = new TankDrive(hardwareMap);

            PosePath forward = drive.pathBuilder(new Position2(0, 0), 0)
                    .forward(DISTANCE)
                    .build();

            PosePath backward = drive.pathBuilder(forward, Math.PI)
                    .forward(-DISTANCE)
                    .build();

            waitForStart();

            while (opModeIsActive()) {
                drive.followPath(forward).runBlocking();
                drive.followPath(backward).runBlocking();
            }
        } else {
            throw new AssertionError();
        }
    }
}
