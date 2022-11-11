package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;

@TeleOp(name = "TeleOpAtlDouble")

public class DoubleDriverTeleOpAtl extends OpMode {
    //wheels
    public DcMotor leftFront = null;
    public DcMotor rightFront = null;
    public DcMotor leftBack = null;
    public DcMotor rightBack = null;
    public double liftPos = 0 ;


    //lift and intake
    public DcMotor lift = null;
    public Servo intake = null;

    public boolean autoLift = false;


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
        if (!autoLift) {
            //lift limits
            liftPos = lift.getCurrentPosition();
            telemetry.addData("lift Power: ", -gamepad2.left_trigger + gamepad2.right_trigger);
            if ((-gamepad2.left_trigger + gamepad2.right_trigger) > 0) {

                if (liftPos < 2000) {
                    telemetry.addData("lift dir: ", "up");
                    lift.setPower(-gamepad2.left_trigger + gamepad2.right_trigger);
                } else {
                    lift.setPower(0);
                }

            } else if ((-gamepad2.left_trigger + gamepad2.right_trigger) < 0) {
                if (liftPos > 20) {
                    telemetry.addData("lift dir: ", "down");
                    lift.setPower(-gamepad2.left_trigger + gamepad2.right_trigger);
                } else {
                    lift.setPower(0);
                }
            } else {
                lift.setPower(0);
            }

            //intake
            if (gamepad2.left_bumper) {
                intake.setPosition(1);
            } else if (gamepad2.right_bumper) {
                intake.setPosition(0);
            }
            telemetry.addData("lift position: ", liftPos);


        } else {
            //auto lift
            if (gamepad2.a) {
                lift.setTargetPosition(1);
                if (lift.getCurrentPosition() > 1) {
                    lift.setPower(-0.5);
                } else {
                    lift.setPower(0.5);
                }

            }else if (gamepad2.b) {
                lift.setTargetPosition(1);
                if (lift.getCurrentPosition() > 1) {
                    lift.setPower(-0.5);
                } else {
                    lift.setPower(0.5);
                }
            }else if (gamepad2.x) {
                lift.setTargetPosition(1);
                if (lift.getCurrentPosition() > 1) {
                    lift.setPower(-0.5);
                } else {
                    lift.setPower(0.5);
                }
            }else if (gamepad2.y) {
                lift.setTargetPosition(1);
                if (lift.getCurrentPosition() > 1) {
                    lift.setPower(-0.5);
                } else {
                    lift.setPower(0.5);
                }
            }

            telemetry.addData("lift position: ", lift.getCurrentPosition());
        }

        if (gamepad2.dpad_down) {
            autoLift = false;
        } else if (gamepad2.dpad_up) {
            autoLift = true;
        }

        //intake
        if (gamepad1.left_bumper) {
            intake.setPosition(1);
        } else if (gamepad1.right_bumper) {
            intake.setPosition(0);
        }

        telemetry.addData("Auto Lift: ", autoLift);
        //intake
        intake.setPosition(gamepad2.left_trigger);
        telemetry.addData("servo state: ", intake.getPosition());

        telemetry.update();

    }
}
