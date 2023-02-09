package org.firstinspires.ftc.teamcode.Tele;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;

@TeleOp(name = "DoubleAtlIntakeFIx")

public class DoubleIntakeFixTest extends OpMode {
    //wheels
    public DcMotor leftFront = null;
    public DcMotor rightFront = null;
    public DcMotor leftBack = null;
    public DcMotor rightBack = null;


    public double liftPos;

    //lift and intake
    public DcMotor lift = null;
    public Servo intake = null;

    public int slowModePressed = 0;
    public boolean slowMode;
    public int liftTarget = 0;
    public double powerProportion = 1.4;


    @Override
    public void init() {
        //wheel motor hardware map
        leftFront = hardwareMap.get(DcMotor.class, "leftFront");
        rightFront = hardwareMap.get(DcMotor.class, "rightFront");
        leftBack = hardwareMap.get(DcMotor.class, "leftBack");
        rightBack = hardwareMap.get(DcMotor.class, "rightBack");

        leftFront.setDirection(DcMotor.Direction.REVERSE);
        rightFront.setDirection(DcMotor.Direction.FORWARD);
        leftBack.setDirection(DcMotor.Direction.REVERSE);
        rightBack.setDirection(DcMotor.Direction.FORWARD);

        leftFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        leftBack.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightBack.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);


        //Lift and intake hardware map
        lift = hardwareMap.get(DcMotor.class, "lift");
        intake = hardwareMap.get(Servo.class, "intake");

        lift.setDirection(DcMotor.Direction.FORWARD);

        lift.setTargetPosition(liftTarget);
        lift.setMode(DcMotor.RunMode.RUN_TO_POSITION);

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
            double turn = (gamepad1.left_stick_x * 0.7);


           /*strafe equation
           leftFrontPower = (straight - strafing - turn);
           rightFrontPower = (straight + strafing + turn);
           leftBackPower = (straight + strafing - turn);
           rightBackPower = (straight - strafing + turn);
*/
            powerProportion = 1.4;
            if (liftPos > 2000) {
                leftFrontPower = ((straight - strafing - turn) * (powerProportion - (liftPos/4500)));
                rightFrontPower = ((straight + strafing + turn) * (powerProportion - (liftPos/4500)));
                leftBackPower = ((straight + strafing - turn) * (powerProportion - (liftPos/4500)));
                rightBackPower = ((straight - strafing + turn) * (powerProportion - (liftPos/4500)));
                slowMode = false;
                // motorPower * (1 - (liftPos/4500))
            } else {
                leftFrontPower = (straight - strafing - turn);
                rightFrontPower = (straight + strafing + turn);
                leftBackPower = (straight + strafing - turn);
                rightBackPower = (straight - strafing + turn);
            }


            //strafe chassis wheel move
            if (slowMode || liftPos > 1500) {
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
                    liftTarget = 1800;
                } else if (gamepad2.x) {
                    liftTarget = 2750;
                } else if (gamepad2.y) {
                    liftTarget = 4500;
                }
            }

            //limits
            if (liftTarget > 4500) {
                liftTarget = 4500;
            } else if (liftTarget < 0) {
                liftTarget = 0;
            }

            //power setting
            lift.setTargetPosition(liftTarget);

            if (liftTarget > liftPos) {
                lift.setPower(1);
            } else if (liftPos > liftTarget) {
                lift.setPower(-.8);
            } else if (liftPos == liftTarget) {
                lift.setPower(0);
            }
        }

        //intake
        telemetry.addData("rightBumper: ", gamepad2.right_bumper);
        {
            if ((gamepad2.left_bumper || gamepad2.right_bumper)) {
                intake.setPosition(0.6);
            }else{
                intake.setPosition(0);
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
        telemetry.addData("Lift Error", liftTarget-liftPos);
        telemetry.update();

    }
}

