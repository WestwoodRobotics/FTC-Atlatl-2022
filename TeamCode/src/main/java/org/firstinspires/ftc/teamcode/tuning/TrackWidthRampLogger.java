package org.firstinspires.ftc.teamcode.tuning;

import com.acmerobotics.roadrunner.Rotation2;
import com.acmerobotics.roadrunner.Transform2;
import com.acmerobotics.roadrunner.Vector2;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorEx;

import org.firstinspires.ftc.robotcore.external.navigation.AngularVelocity;
import org.firstinspires.ftc.teamcode.MecanumDrive;
import org.firstinspires.ftc.teamcode.util.Encoder;
import org.firstinspires.ftc.teamcode.util.MidpointTimer;

import java.util.ArrayList;
import java.util.List;

// FIXME: which group, if any?
@TeleOp(group = "drive")
public final class TrackWidthRampLogger extends LinearOpMode {
    // TODO: control the power function with parameters
    private static double power(double seconds) {
        return Math.min(0.1 * seconds, 0.9);
    }

    @Override
    public void runOpMode() throws InterruptedException {
        DriveView view = new DriveView(new MecanumDrive(hardwareMap,
                new Transform2(new Vector2(0, 0), Rotation2.exp(0))));

        List<Encoder> encs = new ArrayList<>();
        encs.addAll(view.leftEncs);
        encs.addAll(view.rightEncs);
        encs.addAll(view.parEncs);

        // Regression 1: per wheel, commanded voltage against encoder linear velocity
        //   requires powers, voltages, positions, and velocities
        // Regression 2: per robot, encoder angular velocity against IMU angular velocity (roughly)
        //   requires positions, encoder velocities, and IMU angular velocities
        // TODO: log current to improve Regression 1?
        class Data {
            final List<List<Double>> leftPowerTimes = new ArrayList<>();
            final List<List<Double>> leftPowers = new ArrayList<>();

            final List<List<Double>> rightPowerTimes = new ArrayList<>();
            final List<List<Double>> rightPowers = new ArrayList<>();

            final List<Double> voltageTimes = new ArrayList<>();
            final List<Double> voltages = new ArrayList<>();

            final List<Double> encTimes = new ArrayList<>();
            final List<List<Integer>> encPositions = new ArrayList<>();
            final List<List<Integer>> encVels = new ArrayList<>();

            final List<Double> angVelTimes = new ArrayList<>();
            final List<List<Double>> angVels = new ArrayList<>();
        }

        Data data = new Data();
        for (DcMotorEx m : view.leftMotors) {
            data.leftPowerTimes.add(new ArrayList<>());
            data.leftPowers.add(new ArrayList<>());
        }
        for (DcMotorEx m : view.rightMotors) {
            data.rightPowerTimes.add(new ArrayList<>());
            data.rightPowers.add(new ArrayList<>());
        }
        for (Encoder e : encs) {
            data.encPositions.add(new ArrayList<>());
            data.encVels.add(new ArrayList<>());
        }
        for (int i = 0; i < 3; i++) {
            data.angVels.add(new ArrayList<>());
        }

        waitForStart();

        MidpointTimer t = new MidpointTimer();
        while (opModeIsActive()) {
            for (int i = 0; i < view.leftMotors.size(); i++) {
                double power = -power(t.seconds());
                view.leftMotors.get(i).setPower(power);

                data.leftPowers.get(i).add(power);
                data.leftPowerTimes.get(i).add(t.addSplit());
            }

            for (int i = 0; i < view.rightMotors.size(); i++) {
                double power = power(t.seconds());
                view.rightMotors.get(i).setPower(power);

                data.rightPowers.get(i).add(power);
                data.rightPowerTimes.get(i).add(t.addSplit());
            }

            data.voltages.add(view.voltageSensor.getVoltage());
            data.voltageTimes.add(t.addSplit());

            for (int i = 0; i < encs.size(); i++) {
                Encoder.PositionVelocityPair p = encs.get(i).getPositionAndVelocity();
                data.encPositions.get(i).add(p.position);
                data.encVels.get(i).add(p.velocity);
            }
            data.encTimes.add(t.addSplit());

            AngularVelocity av = view.imu.getAngularVelocity();
            data.angVels.get(0).add((double) av.xRotationRate);
            data.angVels.get(1).add((double) av.yRotationRate);
            data.angVels.get(2).add((double) av.zRotationRate);
            data.angVelTimes.add(t.addSplit());
        }

        for (DcMotorEx m : view.leftMotors) {
            m.setPower(0);
        }
        for (DcMotorEx m : view.rightMotors) {
            m.setPower(0);
        }

        TuningFiles.save(TuningFiles.FileType.TRACK_WIDTH_RAMP, data);
    }
}
