/*
package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;

//how you tell program where your going to find it in the driver-station map can be in @teleOp or @Autonomous
//teleOp is controls, Autonomous is the Autonomous period code
@Autonomous(name="motor1")

public class Tutorial extends OpMode {
    //defines motor
    DcMotor motor = null;
    
    //defines servo
    Servo Intake = null;


    //runs after initialization
    @Override
    public void init() {
        //how to go from config file to how to check which port it is in
        motor = hardwareMap.get(DcMotor.class,"motor");
        
        //to set direction of tourque
        motor.setDirection(DcMotor.Direction.REVERSE)
    }


    //loops after start
    @Override
    public void loop() {
    
        
        
        //sets power; 1 is highest, 0 is off, -1 is highest reversed
        motor.setPower(1);


        //allows motor to get power when leftTrigger is pressed
        motor.setPower(gamepad1.leftTrigger)
        
        //uses y movement of left joystick to control motor power
        motor.setPower(gmaepad1.left_stick_y)
        //gmaepad1.left_stick_y or any other gamepad input can be used as a variable so that you don't have to type as much
       
        
        
    }
}




 */
