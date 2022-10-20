package org.firstinspires.ftc.teamcode.drive.opmode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

@TeleOp (name = "strafe")

public class StrafeCodeTest extends  OpMode {
    //wheels
    DcMotor leftFront = null;
    DcMotor rightFront = null;
    DcMotor leftBack = null;
    DcMotor rightBack = null;

    @Override
    public void init() {
        //wheel motor hardware map
        leftFront = hardware.map("leftFront");
        rightFront = hardware.map("rightFront");
        leftBack = hardware.map("leftBack");
        rightBack = hardware.map("rightBack");

    }

    @Override
    public void loop() {
        //defining Wheel power
        double leftFrontPower;
        double rightFrontPower;
        double leftBackPower;
        double rightBackPower;
        
        //giving controls a value to use for drive
        double straight = gamepad1.right_stick_y;
        double strafing = gamepad1.right_stick_x;
        double turn = gamepad1.left_stick_x;

        //strafe equation
        leftFrontPower = (straight + strafing + turn);
        rightFrontPower = (straight - strafing - turn);
        leftBackPower = (straight + strafing + turn);
        rightBackPower = (straight - strafing - turn);

        //strafe chassis wheel move
        leftFront.setPower(leftFrontPower);
        rightFront.setPower(rightFrontPower);
        leftBack.setPower(leftBackPower);
        rightBack.setPower(rightBackPower);
    }
}
