package org.firstinspires.ftc.teamcode.Tests;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;


@TeleOp(name = "Intake Test")
public class IntakeTest extends OpMode {
    public Servo claw1 = null;
    public Servo claw2 = null;
    
    @Override
    public void init() {
        claw1 = hardwareMap.get(Servo.class, "claw1");
        claw2 = hardwareMap.get(Servo.class, "claw2");
    }

    @Override
    public void loop() {
        if(gamepad1.right_bumper || gamepad1.left_bumper){
            claw1.setPosition(1);
            claw2.setPosition(0);
            telemetry.addData("claw", "open");
        }else{
            claw1.setPosition(0);
            claw2.setPosition(1);
            telemetry.addData("claw", "closed");
        }
    }
}
