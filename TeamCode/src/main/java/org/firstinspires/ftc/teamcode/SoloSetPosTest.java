package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Servo;

@TeleOp(name = "set pos test")

public class SoloSetPosTest extends OpMode {
    //wheels
    public DcMotorEx leftFront = null;
    public DcMotorEx rightFront = null;
    public DcMotorEx leftBack = null;
    public DcMotorEx rightBack = null;
    public double liftPos;

    //lift and intake
    public DcMotorEx lift = null;
    public Servo intake = null;

    public int intakePressed = 0;
    public int slowModePressed = 0;
    public boolean slowMode;

    public int liftTarget;
    @Override
    public void init() {
        //wheel motor hardware map
        leftFront = hardwareMap.get(DcMotorEx.class, "leftFront");
        rightFront = hardwareMap.get(DcMotorEx.class, "rightFront");
        leftBack = hardwareMap.get(DcMotorEx.class, "leftBack");
        rightBack = hardwareMap.get(DcMotorEx.class, "rightBack");

        leftFront.setDirection(DcMotorEx.Direction.REVERSE);
        rightFront.setDirection(DcMotorEx.Direction.REVERSE);
        leftBack.setDirection(DcMotorEx.Direction.REVERSE);
        rightBack.setDirection(DcMotorEx.Direction.REVERSE);


        //Lift and intake hardware map
        lift = hardwareMap.get(DcMotorEx.class, "lift");
        intake = hardwareMap.get(Servo.class, "intake");

        lift.setDirection(DcMotorEx.Direction.FORWARD);
        intake.setDirection(Servo.Direction.REVERSE);

        lift.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        lift.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        lift.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);

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
        if (slowMode) {
            leftFront.setPower(leftFrontPower * 0.4);
            rightFront.setPower(rightFrontPower * 0.4);
            leftBack.setPower(leftBackPower * 0.4);
            rightBack.setPower(rightBackPower * 0.4);
        } else {
            leftFront.setPower(leftFrontPower);
            rightFront.setPower(rightFrontPower);
            leftBack.setPower(leftBackPower);
            rightBack.setPower(rightBackPower);
        }


        //lift
        liftPos = lift.getCurrentPosition();

        //manual
        if (!(gamepad1.right_trigger-gamepad1.left_trigger == 0)) {
            /*
            if (gamepad1.right_trigger > 0) {
                liftTarget += 20;
            } else if (gamepad1.left_trigger > 0) {
                liftTarget -= 20;
            }
             */
            liftTarget += Math.round(gamepad1.right_trigger-gamepad1.left_trigger) * 20;
        } else {
            //auto
            if (gamepad1.a) {
                liftTarget = 0;
            } else if (gamepad1.b) {
                liftTarget = 1000;
            } else if (gamepad1.x) {
                liftTarget = 2000;
            } else if (gamepad1.y) {
                liftTarget = 3640;
            }
        }

        //limits
        if (liftTarget > 4000) {
            liftTarget = 4000;
        } else if (liftTarget < 0) {
            liftTarget = 0;
        }

        //power setting
        lift.setTargetPosition(liftTarget);

        if (liftTarget > liftPos) {
            lift.setPower(1);
        } else if (liftPos > liftTarget) {
            lift.setPower(-.6);
        } else if (liftPos == liftTarget) {
            lift.setPower(0);
        }


        //intake
        if ((gamepad1.left_bumper) && intakePressed == 0) {
            if (intake.getPosition()==0.7) {
                intake.setPosition(1);
            } else {
                intake.setPosition(0.7);
            }
            intakePressed++;
        }
        if ((!gamepad1.left_bumper) && intakePressed > 0) {
            intakePressed = 0;
        }
        //slow mode
        if ((gamepad1.right_bumper) && slowModePressed == 0) {
            slowMode = !slowMode;
            slowModePressed++;
        }
        if ((!gamepad1.right_bumper) && slowModePressed > 0) {
            slowModePressed = 0;
        }


        telemetry.addData("slow Mode", slowMode);
        //telemetry
        telemetry.addData("lift position: ", liftPos);
        telemetry.addData("servo state: ", intake.getPosition());
        telemetry.update();

    }
}