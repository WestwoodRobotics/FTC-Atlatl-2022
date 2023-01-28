package org.firstinspires.ftc.teamcode.Tele;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotorEx;

public class tutorial extends OpMode {
    public DcMotorEx goofymotor = null;
    @Override
    public void init() {
        goofymotor = hardwareMap.get(DcMotorEx.class, "goofymotor");
    }

    @Override
    public void loop() {

        if(gamepad1.a){
            goofymotor.setPower(1);
        }else{
            goofymotor.setPower(0);
        }
    }
}
