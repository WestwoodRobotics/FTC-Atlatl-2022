package org.firstinspires.ftc.teamcode.tuning;

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
        MecanumDrive drive = new MecanumDrive(hardwareMap);

        MecanumDrive.DriveLocalizer dl = (MecanumDrive.DriveLocalizer) drive.localizer;

        drive.leftFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        drive.leftBack.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        drive.rightBack.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        drive.rightFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);

        waitForStart();

        double initLateralSum = lateralSum(dl);
        while (opModeIsActive()) {
            telemetry.addData("ticks traveled", lateralSum(dl) - initLateralSum);
            telemetry.update();
        }
    }
}
