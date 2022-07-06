package org.firstinspires.ftc.teamcode.tuning;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.Rotation2;
import com.acmerobotics.roadrunner.Transform2;
import com.acmerobotics.roadrunner.Vector2;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.util.Encoder;
import org.firstinspires.ftc.teamcode.MecanumDrive;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.List;

@Config
@Autonomous(group = "drive")
// FIXME: capture angular velocity data from all axes and help the user figure out any remapping
public class TrackWidthLogger extends LinearOpMode {
    private static double power(double seconds) {
        return 0.5 * Math.sin(1e-9 * seconds);
    }

    @Override
    public void runOpMode() throws InterruptedException {
        Object drive = new MecanumDrive(hardwareMap, new Transform2(new Vector2(0, 0), Rotation2.exp(0)));
        DriveView view = new DriveView(drive);

        // FIXME: maybe just move this to view anyway
        final List<Encoder> leftEncs, rightEncs;
        if (drive instanceof MecanumDrive) {
            MecanumDrive md = (MecanumDrive) drive;
            if (md.localizer instanceof MecanumDrive.DriveLocalizer) {
                MecanumDrive.DriveLocalizer dl = (MecanumDrive.DriveLocalizer) md.localizer;
                leftEncs = Arrays.asList(dl.leftFront, dl.leftRear);
                rightEncs = Arrays.asList(dl.rightFront, dl.rightRear);
            } else {
                // TODO: error
                throw new RuntimeException();
            }
        }
        // TODO: handle tank
        else {
            throw new RuntimeException();
        }

        class Data {
            final String type = view.type;
            final List<Double> angVelTimes = new ArrayList<>();
            final List<Double> angVels = new ArrayList<>();
            final List<Double> encVelTimes = new ArrayList<>();
            final List<List<Integer>> leftEncVels = new ArrayList<>();
            final List<List<Integer>> rightEncVels = new ArrayList<>();
        }
        Data data = new Data();
        for (Encoder e : leftEncs) {
            data.leftEncVels.add(new ArrayList<>());
        }
        for (Encoder e : rightEncs) {
            data.rightEncVels.add(new ArrayList<>());
        }

        ElapsedTime t = new ElapsedTime();
        while (opModeIsActive()) {
            for (DcMotorEx m : view.leftMotors) {
                m.setPower(-power(t.seconds()));
            }
            for (DcMotorEx m : view.rightMotors) {
                m.setPower(power(t.seconds()));
            }

            double t0 = t.seconds();

            for (int i = 0; i < leftEncs.size(); i++) {
                data.leftEncVels.get(i).add(leftEncs.get(i).getPositionAndVelocity().velocity);
            }
            for (int i = 0; i < rightEncs.size(); i++) {
                data.rightEncVels.get(i).add(rightEncs.get(i).getPositionAndVelocity().velocity);
            }

            double t1 = t.seconds();

            data.angVels.add(view.imu.getHeadingVelocity());

            double t2 = t.seconds();

            data.encVelTimes.add(0.5 * (t0 + t1));
            data.angVelTimes.add(0.5 * (t1 + t2));
        }

        TuningFiles.save(TuningFiles.FileType.TRACK_WIDTH, data);
    }
}
