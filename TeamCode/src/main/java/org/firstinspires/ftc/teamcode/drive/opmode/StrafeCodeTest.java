package org.firstinspires.ftc.teamcode.drive.opmode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

@TeleOp (name = "strafe")

public class StrafeCodeTest extends  OpMode {
    DcMotor leftFront;
    DcMotor rightFront;
    DcMotor leftBack;
    DcMotor rightBack;

    @Override
    public void init() {
        leftFront = hardwareMap.dcMotor.get("motor1");
        rightFront = hardwareMap.dcMotor.get("motor2");
        leftBack = hardwareMap.dcMotor.get("motor3");
        rightBack = hardwareMap.dcMotor.get("motor4");

    }

    @Override
    public void loop() {
        double leftFrontPower;
        double rightFrontPower;
        double leftBackPower;
        double rightBackPower;

        double straight = gamepad1.right_stick_y;
        double strafing = gamepad1.right_stick_x;
        double turn = gamepad1.left_stick_x;

        leftFrontPower = (straight + strafing + turn);
        rightFrontPower = (straight - strafing - turn);
        leftBackPower = (straight + strafing + turn);
        rightBackPower = (straight - strafing - turn);



        leftFront.setPower(leftFrontPower);
        rightFront.setPower(rightFrontPower);
        leftBack.setPower(leftBackPower);
        rightBack.setPower(rightBackPower);
    }
}
