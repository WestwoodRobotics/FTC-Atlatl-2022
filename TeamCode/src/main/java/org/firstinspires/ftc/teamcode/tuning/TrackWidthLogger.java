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

@TeleOp(group = "drive")
// FIXME: capture angular velocity data from all axes and help the user figure out any remapping
public class TrackWidthLogger extends LinearOpMode {
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
            final List<Double> angVels = new ArrayList<>();
            final List<Double> encVelTimes = new ArrayList<>();
            final List<List<Integer>> leftEncVels = new ArrayList<>();
            final List<List<Integer>> rightEncVels = new ArrayList<>();
        }
        Data data = new Data();
        for (Encoder e : view.leftEncs) {
            data.leftEncVels.add(new ArrayList<>());
        }
        for (Encoder e : view.rightEncs) {
            data.rightEncVels.add(new ArrayList<>());
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
            for (int i = 0; i < view.leftEncs.size(); i++) {
                data.leftEncVels.get(i).add(view.leftEncs.get(i).getPositionAndVelocity().velocity);
            }
            for (int i = 0; i < view.rightEncs.size(); i++) {
                data.rightEncVels.get(i).add(view.rightEncs.get(i).getPositionAndVelocity().velocity);
            }
            data.encVelTimes.add(t.addSplit());

            data.angVels.add(view.imu.getHeadingVelocity());
            data.angVelTimes.add(t.addSplit());
        }

        TuningFiles.save(TuningFiles.FileType.TRACK_WIDTH, data);
    }
}
