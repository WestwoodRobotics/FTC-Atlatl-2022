package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;

@Autonomous(name = "encoder")
public class ENcoderTEst extends OpMode {
    public DcMotor motor1 = null;
    @Override
    public void init() {
        motor1 = hardwareMap.get(DcMotor.class, "motor");
        motor1.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motor1.setTargetPosition(400);
        motor1.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        motor1.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

    }

    @Override
    public void loop() {
        motor1.setPower(1);
        telemetry.addData("emc",motor1.getCurrentPosition());
        telemetry.update();
    }
}
