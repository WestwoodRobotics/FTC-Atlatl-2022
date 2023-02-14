package org.firstinspires.ftc.teamcode.Tests;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

@TeleOp(name = "Lift encoders")
public class LiftPositionDirectionTest extends OpMode {

    public DcMotor lift = null;
    @Override
    public void init() {
        //Lift and intake hardware map
        lift = hardwareMap.get(DcMotor.class, "lift");

        lift.setDirection(DcMotor.Direction.FORWARD);
        lift.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        lift.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    }

    @Override
    public void loop() {
        telemetry.addData("lift encoder Pos",  lift.getCurrentPosition());
        telemetry.update();
    }
}
