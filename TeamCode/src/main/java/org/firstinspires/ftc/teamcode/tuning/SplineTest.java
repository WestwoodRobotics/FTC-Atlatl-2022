package org.firstinspires.ftc.teamcode.tuning;

import com.acmerobotics.roadrunner.Position2d;
import com.acmerobotics.roadrunner.Transform2d;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.MecanumDrive;
import org.firstinspires.ftc.teamcode.TankDrive;

public final class SplineTest extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        if (TuningOpModes.DRIVE_CLASS.equals(MecanumDrive.class)) {
            MecanumDrive drive = new MecanumDrive(hardwareMap);

            waitForStart();

            drive.followPath(
                    drive.pathBuilder(new Transform2d(0, 0, 0), 0)
                            .splineTo(new Position2d(30, 30), Math.PI / 2)
                            .splineTo(new Position2d(60, 0), Math.PI)
                            .build()
            ).runBlocking();
        } else if (TuningOpModes.DRIVE_CLASS.equals(TankDrive.class)) {
            TankDrive drive = new TankDrive(hardwareMap);

            waitForStart();

            drive.followPath(
                    drive.pathBuilder(new Position2d(0, 0), 0)
                            .splineTo(new Position2d(30, 30), Math.PI / 2)
                            .splineTo(new Position2d(60, 0), Math.PI)
                            .build()
            ).runBlocking();
        } else {
            throw new AssertionError();
        }
    }
}
