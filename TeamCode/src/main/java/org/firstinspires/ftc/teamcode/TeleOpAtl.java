package org.firstinspires.ftc.teamcode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;

@TeleOp(name = "TeleOpAtl")

public class TeleOpAtl extends OpMode {
    //wheels
    DcMotor leftFront = null;
    DcMotor rightFront = null;
    DcMotor leftBack = null;
    DcMotor rightBack = null;


    //lift and intake
    DcMotor lift = null;
    Servo intake = null;
    public boolean AutoLift = true;


    @Override
    public void init() {
        //wheel motor hardware map
        leftFront = hardwareMap.get(DcMotor.class,"leftFront");
        rightFront = hardwareMap.get(DcMotor.class,"rightFront");
        leftBack = hardwareMap.get(DcMotor.class,"leftBack");
        rightBack = hardwareMap.get(DcMotor.class,"rightBack");

        leftFront.setDirection(DcMotor.Direction.FORWARD);
        rightFront.setDirection(DcMotor.Direction.REVERSE);
        leftBack.setDirection(DcMotor.Direction.FORWARD);
        rightBack.setDirection(DcMotor.Direction.REVERSE);

        leftFront.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightFront.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        leftBack.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightBack.setMode(DcMotor.RunMode.RUN_USING_ENCODER);


        //Lift and intake hardware map
        lift = hardwareMap.get(DcMotor.class, "lift");
        intake = hardwareMap.get(Servo.class, "intake");

        lift.setDirection(DcMotor.Direction.FORWARD);
        intake.setDirection(Servo.Direction.REVERSE);

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


        //lift & intake
        if (gamepad2.left_bumper){
            while (gamepad2.left_bumper){
                telemetry.addData("ERROR: ","LET GO OF THE LEFT BUMPER IDIOT");
            }
            AutoLift = !AutoLift;
            telemetry.addData("autoLift: ", AutoLift);
        }

        if (AutoLift){
            if (gamepad2.a){
                telemetry.addData("button A: ","pressed");
            }
            if (gamepad2.b){
                telemetry.addData("button B: ","pressed");
            }
            if (gamepad2.x){
                telemetry.addData("button X: ","pressed");
            }
            if (gamepad2.y){
                telemetry.addData("button Y: ","pressed");
            }
        } else {lift.setPower(gamepad2.left_stick_y);}
    }
}