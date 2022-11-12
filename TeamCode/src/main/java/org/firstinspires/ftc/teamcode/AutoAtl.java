package org.firstinspires.ftc.teamcode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;

@Autonomous(name = "AutoAtl")

public class AutoAtl extends OpMode {
    //wheels
    public DcMotor leftFront = null;
    public DcMotor rightFront = null;
    public DcMotor leftBack = null;
    public DcMotor rightBack = null;

    //lift and intake
    public DcMotor lift = null;
    public Servo intake = null;

    @Override
    public void init() {
        //wheel motor hardware map
        leftFront = hardwareMap.get(DcMotor.class,"leftFront");
        rightFront = hardwareMap.get(DcMotor.class,"rightFront");
        leftBack = hardwareMap.get(DcMotor.class,"leftBack");
        rightBack = hardwareMap.get(DcMotor.class,"rightBack");

        leftFront.setDirection(DcMotor.Direction.REVERSE);
        rightFront.setDirection(DcMotor.Direction.REVERSE);
        leftBack.setDirection(DcMotor.Direction.REVERSE);
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
        leftFront.setTargetPosition(300);
        rightFront.setTargetPosition(300);
        leftBack.setTargetPosition(300);
        rightBack.setTargetPosition(300);

        leftFront.setPower(1);
        rightFront.setPower(1);
        leftBack.setPower(1);
        rightBack.setPower(1);


    }
}
