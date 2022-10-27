package org.firstinspires.ftc.teamcode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;

@TeleOp(name = "TeleOpAtl")

public class TeleOpAtl extends OpMode {
    //wheels
    public DcMotor leftFront = null;
    public DcMotor rightFront = null;
    public DcMotor leftBack = null;
    public DcMotor rightBack = null;


    //lift and intake
    public DcMotor lift = null;
    public Servo intake = null;
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

        lift.setMode(DcMotor.RunMode.RUN_TO_POSITION);
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

        //defining AutoLift buttons
        boolean bottom;
        boolean low;
        boolean mid;
        boolean high;

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

        //AutoLift buttons
        bottom = gamepad2.a;
        low = gamepad2.b;
        mid = gamepad2.x;
        high = gamepad2.y;


        //lift & intake
        if (AutoLift){
            lift.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        }else{
            lift.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        }

        if (gamepad2.left_bumper){
            AutoLift = !AutoLift;
            telemetry.addData("autoLift: ", AutoLift);
            while (gamepad2.left_bumper){
                telemetry.addData("ERROR: ","LET GO OF THE LEFT BUMPER IDIOT");
            }
        }

        if (AutoLift){
            if (bottom){
                telemetry.addData("button A: ","pressed");
                while (gamepad2.a){
                    telemetry.addData("ERROR: ","LET GO OF THE A BUTTON IDIOT");
                }
            }
            if (low){
                telemetry.addData("button B: ","pressed");
                while (gamepad2.b){
                    telemetry.addData("ERROR: ","LET GO OF THE B BUTTON IDIOT");
                }

            }
            if (mid){
                while (gamepad2.x){
                    telemetry.addData("button X: ","pressed");
                    telemetry.addData("ERROR: ","LET GO OF THE X BUTTON IDIOT");
                }
            }
            if (high){
                telemetry.addData("button Y: ","pressed");
                while (gamepad2.y){
                    telemetry.addData("ERROR: ","LET GO OF THE Y BUTTON IDIOT");
                }

            }
        } else {lift.setPower(gamepad2.left_stick_y);}
    }
}