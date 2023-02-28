package org.firstinspires.ftc.teamcode.Tests;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.arcrobotics.ftclib.controller.PIDController;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorEx;

@Config
@TeleOp
public class LiftPIDF extends OpMode {
    private PIDController controller;

    public static double p = 0, i = 0, d = 0;
    public static double f = 0.06;

    public static int target = 0;
    private final double ticks_per_degree = 384.5/360;

    private DcMotorEx lift = null;

    @Override
    public void init() {
        controller = new PIDController(p,i,d);
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        lift = hardwareMap.get(DcMotorEx.class, "lift");
        lift.setDirection(DcMotorEx.Direction.REVERSE);

        lift.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        lift.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);


    }

    @Override
    public void loop() {
        controller.setPID(p, i, d);
        int liftPos             = lift.getCurrentPosition();
        double pid = controller.calculate(liftPos, target);

        double ff = Math.cos(Math.toRadians(target/ticks_per_degree)) * f;

        double power = pid + ff;

        lift.setPower(power);

        telemetry.addData("lift position", liftPos);
        telemetry.addData("lift target", target);
        telemetry.update();
    }
}
