package org.firstinspires.ftc.teamcode.tuning;

import com.acmerobotics.roadrunner.Rotation2;
import com.acmerobotics.roadrunner.Transform2;
import com.acmerobotics.roadrunner.Vector2;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorEx;

import org.firstinspires.ftc.teamcode.MecanumDrive;
import org.firstinspires.ftc.teamcode.util.Encoder;
import org.firstinspires.ftc.teamcode.util.MidpointTimer;

import java.util.ArrayList;
import java.util.List;

// FIXME: which group, if any?
@TeleOp(group = "drive")
public final class AccelLogger extends LinearOpMode {
    private static final double POWER = 0.8;

    @Override
    public void runOpMode() throws InterruptedException {
        DriveView view = new DriveView(new MecanumDrive(hardwareMap,
                new Transform2(new Vector2(0, 0), Rotation2.exp(0))));

        List<DcMotorEx> motors = new ArrayList<>();
        motors.addAll(view.leftMotors);
        motors.addAll(view.rightMotors);

        List<Encoder> encs = new ArrayList<>();
        encs.addAll(view.leftEncs);
        encs.addAll(view.rightEncs);
        encs.addAll(view.parEncs);

        // FIXME: log current
        class Data {
            final List<List<Double>> powerTimes = new ArrayList<>();
            final List<List<Double>> powers = new ArrayList<>();
            final List<Double> voltageTimes = new ArrayList<>();
            final List<Double> voltages = new ArrayList<>();
            final List<Double> encVelTimes = new ArrayList<>();
            final List<List<Integer>> encVels = new ArrayList<>();
        }
        Data data = new Data();
        for (DcMotorEx m : motors) {
            data.powerTimes.add(new ArrayList<>());
            data.powers.add(new ArrayList<>());
        }
        for (Encoder e : encs) {
            data.encVels.add(new ArrayList<>());
        }

        waitForStart();

        MidpointTimer t = new MidpointTimer();
        for (DcMotorEx m : motors) {
            m.setPower(POWER);
        }

        while (opModeIsActive()) {
            for (int i = 0; i < motors.size(); i++) {
                data.powerTimes.get(i).add(t.addSplit());
                data.powers.get(i).add(POWER);
            }

            data.voltages.add(view.voltageSensor.getVoltage());
            data.voltageTimes.add(t.addSplit());

            for (int i = 0; i < encs.size(); i++) {
                data.encVels.get(i).add(
                        encs.get(i).getPositionAndVelocity().velocity);
            }
            data.encVelTimes.add(t.addSplit());
        }

        for (DcMotorEx m : motors) {
            m.setPower(0);
        }

        TuningFiles.save(TuningFiles.FileType.ACCEL, data);
    }
}
