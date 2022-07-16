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

@TeleOp(group = "drive")
// TODO: finish this
// TODO: can we get feedforward ramp data from this and the track width tuner?
public final class DeadWheelLogger extends LinearOpMode {
    private static double power(double seconds) {
        return 0.5 * Math.sin(1e-9 * seconds);
    }

    @Override
    public void runOpMode() throws InterruptedException {
        DriveView view = new DriveView(new MecanumDrive(hardwareMap,
                new Transform2(new Vector2(0, 0), Rotation2.exp(0))));

        class Data {
            final String type = view.type;
            final List<Double> angVelTimes = new ArrayList<>();
            final List<List<Double>> angVels = new ArrayList<>();
            final List<Double> encVelTimes = new ArrayList<>();
            final List<List<Integer>> parEncVels = new ArrayList<>();
            final List<List<Integer>> perpEncVels = new ArrayList<>();
        }
        Data data = new Data();
        for (int i = 0; i < 3; i++) {
            data.angVels.add(new ArrayList<>());
        }
        for (Encoder e : view.parEncs) {
            data.parEncVels.add(new ArrayList<>());
        }
        for (Encoder e : view.perpEncs) {
            data.perpEncVels.add(new ArrayList<>());
        }

        MidpointTimer t = new MidpointTimer();
        while (opModeIsActive()) {
            for (DcMotorEx m : view.leftMotors) {
                m.setPower(-power(t.seconds()));
            }
            for (DcMotorEx m : view.rightMotors) {
                m.setPower(power(t.seconds()));
            }

            t.addSplit();
            for (int i = 0; i < view.parEncs.size(); i++) {
                data.parEncVels.get(i).add(view.parEncs.get(i).getPositionAndVelocity().velocity);
            }
            for (int i = 0; i < view.perpEncs.size(); i++) {
                data.perpEncVels.get(i).add(view.perpEncs.get(i).getPositionAndVelocity().velocity);
            }
            data.encVelTimes.add(t.addSplit());

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

        TuningFiles.save(TuningFiles.FileType.DEAD_WHEELS, data);
    }
}
