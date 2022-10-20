/*
package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;

//how you tell program where your going to find it in the driver-station map can be in @teleOp or @Autonomous
//teleOp is controls, Autonomous is the Autonomous period code
@Autonomous(name="motor1")

public class Tutorial extends OpMode {

    DcMotor motor;


    //runs after initialization
    @Override
    public void init() {
        //how to go from config file to how to check which port it is in
        motor = hardwareMap.dcMotor.get("motor1");
        or
        motor = hardwareMap.get(DcMotor.class, deviceName: "motor1");
        motor = hardwareMap.dcMotor.get("motor1");
    }


    //loops after start
    @Override
    public void loop() {
        //sets power
        motor.setPower(1);


        //allows motor to get power when leftTrigger is pressed
        motor.setPower(gamepad1.leftTrigger)
    }
}





 */
