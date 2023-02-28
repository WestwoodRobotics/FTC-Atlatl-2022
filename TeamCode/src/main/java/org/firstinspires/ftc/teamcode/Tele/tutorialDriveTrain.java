package org.firstinspires.ftc.teamcode.Tele;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

public class tutorialDriveTrain extends OpMode {
    private DcMotor frontRight = null;
    private DcMotor frontLeft = null;
    private DcMotor backRight = null;
    private DcMotor backLeft = null;

    @Override
    public void init() {
        frontRight = hardwareMap.get(DcMotor.class, "frontRight");
        frontLeft = hardwareMap.get(DcMotor.class, "frontLeft");
        backRight = hardwareMap.get(DcMotor.class, "backRight");
        backLeft= hardwareMap.get(DcMotor.class, "backLeft");

        frontRight.setDirection(DcMotor.Direction.FORWARD);
        frontLeft.setDirection(DcMotor.Direction.FORWARD);
        backRight.setDirection(DcMotor.Direction.FORWARD);
        backLeft.setDirection(DcMotor.Direction.FORWARD);

    }

    @Override
    public void loop() {
        frontRight.setPower(gamepad1.left_stick_y);
        frontLeft.setPower(gamepad1.left_stick_y);
        backRight.setPower(gamepad1.left_stick_y);
        backLeft.setPower(gamepad1.left_stick_y);
    }
}
