package org.firstinspires.ftc.teamcode.tuning;

import com.acmerobotics.roadrunner.Rotation2;
import com.acmerobotics.roadrunner.Transform2;
import com.acmerobotics.roadrunner.Vector2;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.teamcode.MecanumDrive;

@TeleOp
public final class LateralPushTest extends LinearOpMode {
    private static double lateralSum(MecanumDrive.DriveLocalizer dl) {
        return 0.25 * (
                -dl.leftFront.getPositionAndVelocity().position
                +dl.leftRear.getPositionAndVelocity().position
                -dl.rightRear.getPositionAndVelocity().position
                +dl.rightFront.getPositionAndVelocity().position);
    }

    @Override
    public void runOpMode() throws InterruptedException {
        MecanumDrive drive = new MecanumDrive(hardwareMap,
                new Transform2(new Vector2(0, 0), Rotation2.exp(0)));

        MecanumDrive.DriveLocalizer dl = (MecanumDrive.DriveLocalizer) drive.localizer;

        drive.leftFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        drive.leftRear.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        drive.rightRear.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        drive.rightFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);

        waitForStart();

        double initLateralSum = lateralSum(dl);
        while (opModeIsActive()) {
            telemetry.addData("ticks traveled", lateralSum(dl) - initLateralSum);
            telemetry.update();
        }
    }
}
