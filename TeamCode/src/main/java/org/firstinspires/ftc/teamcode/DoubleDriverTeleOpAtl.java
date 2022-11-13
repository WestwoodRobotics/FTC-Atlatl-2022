package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;

@TeleOp(name = "TeleOpAtlSolo")

public class DoubleDriverTeleOpAtl extends OpMode {
    //wheels
    public DcMotor leftFront = null;
    public DcMotor rightFront = null;
    public DcMotor leftBack = null;
    public DcMotor rightBack = null;
    public double liftPos;

    //lift and intake
    public DcMotor lift = null;
    public Servo intake = null;

    public int intakePressed = 0;
    public boolean autoLift = false;
    public double liftPower;
    public int liftTarget = 0;
    public int dpadPressed  = 0;
    public boolean slowMode = false;


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

    }


    @Override
    public void loop() {
        //drive chassis
        {
            slowMode = gamepad1.left_bumper;

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
            if ((liftPos > 2000) || slowMode) {
                leftFront.setPower(leftFrontPower * 0.3);
                rightFront.setPower(rightFrontPower * 0.3);
                leftBack.setPower(leftBackPower * 0.3);
                rightBack.setPower(rightBackPower * 0.3);
            } else {
                leftFront.setPower(leftFrontPower);
                rightFront.setPower(rightFrontPower);
                leftBack.setPower(leftBackPower);
                rightBack.setPower(rightBackPower);
            }

        }

        //liftPos variable
        liftPos = lift.getCurrentPosition();

        //autoLift switcher
        {
            if ((gamepad2.dpad_down) && dpadPressed == 0) {
                autoLift = !autoLift;
                dpadPressed++;
            }
            if (!(gamepad2.dpad_down) && dpadPressed > 0) {
                dpadPressed = 0;
            }
        }
        //lift power and position set
        {
            if (autoLift) {
                lift.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                //down
                if (gamepad2.a) {
                    liftTarget = 0;
                }
                //low
                if (gamepad2.b) {
                    liftTarget = 0;
                }
                //mid
                if (gamepad2.x) {
                    liftTarget = 0;
                }
                //high
                if (gamepad2.y) {
                    liftTarget = 0;
                }
                lift.setTargetPosition(liftTarget);

                if (liftPos > liftTarget+25) {
                    //
                    lift.setPower(-1);
                } else if (liftPos < liftTarget-25) {
                    lift.setPower(1);
                }

            } else {
                //manual lift
                lift.setPower(0);
                lift.setMode(DcMotor.RunMode.RUN_USING_ENCODER);


                liftPower = (-gamepad2.left_trigger + gamepad2.right_trigger);
                telemetry.addData("Lift Power: ", liftPower);
                //going up
                if (liftPower > 0) {
                    if (liftPos < 3400) {
                        telemetry.addData("lift dir: ", "up");
                        lift.setPower(liftPower);
                    } else {
                        lift.setPower(0);
                    }

                } else if (liftPower < 0) {
                    if (liftPos > 100) {
                        telemetry.addData("lift dir:", "down");
                        lift.setPower(liftPower);
                    } else {
                        lift.setPower(0);
                    }
                } else {
                    lift.setPower(0);
                }
            }
        }

        //intake
        {
            if ((gamepad2.left_bumper || gamepad2.right_bumper) && intakePressed == 0) {
                if (intake.getPosition() == 0.76) {
                    intake.setPosition(1);
                } else {
                    intake.setPosition(0.76);
                }
                intakePressed++;
            }
            if ((!gamepad2.left_bumper && !gamepad2.right_bumper) && intakePressed > 0) {
                intakePressed = 0;
            }
        }
        //telemetry
        telemetry.addData("lift position: ", liftPos);
        telemetry.addData("servo state: ", intake.getPosition());
        telemetry.update();

    }
}