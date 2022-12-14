package org.firstinspires.ftc.teamcode.Tele;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;

@TeleOp(name = "DoubleAtlTester")

public class DoubleTeleTester extends OpMode {
    //wheels
    public DcMotor leftFront = null;
    public DcMotor rightFront = null;
    public DcMotor leftBack = null;
    public DcMotor rightBack = null;

    public double liftPos;

    //lift and intake
    public DcMotor lift = null;
    //public DcMotor lift2 = null;
    public Servo intake = null;

    public int intakePressed = 0;
    public int slowModePressed = 0;
    public boolean slowMode;
    public int liftTarget = 0;
    public double powerProportion = 0.0;

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

        leftFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        leftBack.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightBack.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);


        //Lift and intake hardware map
        lift = hardwareMap.get(DcMotor.class, "lift");
        //lift2 = hardwareMap.get(DcMotor.class, "secondLift");
        intake = hardwareMap.get(Servo.class, "intake");

        lift.setDirection(DcMotor.Direction.REVERSE);
        //lift2.setDirection(DcMotor.Direction.FORWARD);
        intake.setDirection(Servo.Direction.REVERSE);

        lift.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        lift.setTargetPosition(liftTarget);
        lift.setMode(DcMotor.RunMode.RUN_TO_POSITION);


        //change to commented out if needed
        lift.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

    }


    @Override
    public void loop() {
        //chassis
        {
            //defining Wheel power
            double leftFrontPower;
            double rightFrontPower;
            double leftBackPower;
            double rightBackPower;

            //giving controls a value to use for drive

            double straight = gamepad1.right_stick_y;
            double strafing = gamepad1.right_stick_x;
            double turn = (gamepad1.left_stick_x * 0.8);

            powerProportion = 1.2;

            //strafe equation
            if (liftPos > 500) {
                leftFrontPower = (straight - strafing - turn) * powerProportion * (4000 / liftPos);
                rightFrontPower = (straight + strafing + turn) * powerProportion * (4000 / liftPos);
                leftBackPower = (straight + strafing - turn) * powerProportion * (4000 / liftPos);
                rightBackPower = (straight - strafing + turn) * powerProportion * (4000 / liftPos);
            } else {
                leftFrontPower = (straight - strafing - turn);
                rightFrontPower = (straight + strafing + turn);
                leftBackPower = (straight + strafing - turn);
                rightBackPower = (straight - strafing + turn);
            }
            //strafe chassis wheel move
            leftFront.setPower(leftFrontPower);
            rightFront.setPower(rightFrontPower);
            leftBack.setPower(leftBackPower);
            rightBack.setPower(rightBackPower);

        }

        //lift
        {
            liftPos = lift.getCurrentPosition();
            //manual
            if (!(gamepad2.right_trigger - gamepad2.left_trigger == 0)) {
                liftTarget += Math.round(gamepad2.right_trigger - gamepad2.left_trigger) * 15;
            } else {
                //auto
                if (gamepad2.a) {
                    liftTarget = 0;
                } else if (gamepad2.b) {
                    liftTarget = 1500;

                } else if (gamepad2.x) {
                    liftTarget = 2400;
                } else if (gamepad2.y) {
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
                lift.setPower(-.5);
            } else if (liftPos == liftTarget) {
                lift.setPower(0);
            }
        }

        //intake
        {
            if ((gamepad2.left_bumper || gamepad2.right_bumper) && intakePressed == 0) {
                if (intake.getPosition() == 0.7) {
                    intake.setPosition(1);
                } else {
                    intake.setPosition(0.7);
                }
                intakePressed++;
            }
            if ((!gamepad2.left_bumper && !gamepad2.right_bumper) && intakePressed > 0) {
                intakePressed = 0;
            }
        }

        //slow mode toggle
        {
            if ((gamepad1.right_bumper || gamepad1.left_bumper) && slowModePressed == 0) {
                slowMode = !slowMode;
                slowModePressed++;
            }
            if ((!gamepad1.right_bumper && !gamepad1.left_bumper) && slowModePressed > 0) {
                slowModePressed = 0;
            }
        }


        //telemetry
        telemetry.addData("slow Mode", slowMode);
        telemetry.addData("lift position: ", liftPos);
        telemetry.addData("servo state: ", intake.getPosition());
        telemetry.update();

    }
}

