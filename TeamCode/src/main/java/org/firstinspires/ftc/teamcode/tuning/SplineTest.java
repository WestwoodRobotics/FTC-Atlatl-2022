package org.firstinspires.ftc.teamcode.tuning;

import com.acmerobotics.roadrunner.Position2;
import com.acmerobotics.roadrunner.Transform2;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.MecanumDrive;

public final class SplineTest extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        MecanumDrive drive = new MecanumDrive(hardwareMap);

        waitForStart();

        drive.followPath(
                drive.pathBuilder(new Transform2(0, 0, 0), 0)
                        .splineTo(new Position2(30, 30), Math.PI / 2)
                        .splineTo(new Position2(60, 0), Math.PI)
                        .build()
        ).runBlocking();
    }
}
