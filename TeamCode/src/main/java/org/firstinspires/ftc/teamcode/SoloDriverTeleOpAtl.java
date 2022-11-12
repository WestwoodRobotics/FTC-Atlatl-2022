package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;

@TeleOp(name = "TeleOpAtlSolo")

public class SoloDriverTeleOpAtl extends OpMode {
    //wheels
    public DcMotor leftFront = null;
    public DcMotor rightFront = null;
    public DcMotor leftBack = null;
    public DcMotor rightBack = null;
    public double liftPos;

    //lift and intake
    public DcMotor lift = null;
    public Servo intake = null;


    @Override
    public void init() {
        //wheel motor hardware map
        leftFront = hardwareMap.get(DcMotor.class, "leftFront");
        rightFront = hardwareMap.get(DcMotor.class, "rightFront");
        leftBack = hardwareMap.get(DcMotor.class, "leftBack");
        rightBack = hardwareMap.get(DcMotor.class, "rightBack");

        leftFront.setDirection(DcMotor.Direction.REVERSE);
        rightFront.setDirection(DcMotor.Direction.REVERSE);
        leftBack.setDirection(DcMotor.Direction.REVERSE);
        rightBack.setDirection(DcMotor.Direction.REVERSE);


        //Lift and intake hardware map
        lift = hardwareMap.get(DcMotor.class, "lift");
        intake = hardwareMap.get(Servo.class, "intake");

        lift.setDirection(DcMotor.Direction.FORWARD);
        intake.setDirection(Servo.Direction.REVERSE);

        lift.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        lift.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        lift.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        intake.setPosition(1);
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
        double turn = (gamepad1.left_stick_x * 0.7);


        //strafe equation
        leftFrontPower = (straight - strafing - turn);
        rightFrontPower = (straight + strafing + turn);
        leftBackPower = (straight + strafing - turn);
        rightBackPower = (straight - strafing + turn);

        //strafe chassis wheel move
        leftFront.setPower(leftFrontPower);
        rightFront.setPower(rightFrontPower);
        leftBack.setPower(leftBackPower);
        rightBack.setPower(rightBackPower);


        //lift

        liftPos = lift.getCurrentPosition();
        telemetry.addData("Lift Power: ", -gamepad1.left_trigger + gamepad1.right_trigger);
        if ((-gamepad1.left_trigger + gamepad1.right_trigger) > 0) {

            if (liftPos < 3400) {
                telemetry.addData("lift dir: ", "up");
                lift.setPower((-gamepad1.left_trigger + gamepad1.right_trigger));
            } else {
                lift.setPower(0);
            }

        } else if ((-gamepad1.left_trigger + gamepad1.right_trigger) < 150) {
            if (liftPos > 0) {
                telemetry.addData("lift dir:", "down");
                lift.setPower((-gamepad1.left_trigger + gamepad1.right_trigger));
            } else {
                lift.setPower(0);
            }
        } else {
            lift.setPower(0);
        }

        //intake

        if (gamepad1.left_bumper) {
            intake.setPosition(1);
        } else if (gamepad1.right_bumper){
            intake.setPosition(0.7);
        }

        //telemetry
        telemetry.addData("lift position: ", liftPos);
        telemetry.addData("servo state: ", intake.getPosition());
        telemetry.update();

    }
}