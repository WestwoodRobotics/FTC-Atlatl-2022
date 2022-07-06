package org.firstinspires.ftc.teamcode.tuning;

import com.acmerobotics.roadrunner.Rotation2;
import com.acmerobotics.roadrunner.Transform2;
import com.acmerobotics.roadrunner.Vector2;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;

import org.firstinspires.ftc.teamcode.MecanumDrive;
import org.firstinspires.ftc.teamcode.util.Encoder;

import java.util.List;

// TODO: keep track of individual encoder deltas to warn about reversal?
// pro: less need for motor debugger, forces user to deal with encoders
// con: potentially unreliable (irregular pushing, variable cpr), complicates code

// TODO: place drive and dead routines in different packages?

// TODO: prefix class names with numbers to organize tuning?
// TODO: public class Tune0_ForwardPush extends ...?
@TeleOp
public final class ForwardPushTest extends LinearOpMode {
    private static double avgPos(List<? extends Encoder> es) {
        double avgPos = 0;
        for (Encoder e : es) {
            avgPos += e.getPositionAndVelocity().position;
        }
        return avgPos / es.size();
    }

    @Override
    public void runOpMode() throws InterruptedException {
        DriveView view = new DriveView(new MecanumDrive(hardwareMap,
                new Transform2(new Vector2(0, 0), Rotation2.exp(0))));

        for (DcMotorEx m : view.leftMotors) {
            m.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        }
        for (DcMotorEx m : view.rightMotors) {
            m.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        }

        waitForStart();

        double initAvgPos = avgPos(view.parEncs);
        while (opModeIsActive()) {
            telemetry.addData("ticks traveled", avgPos(view.parEncs) - initAvgPos);
            telemetry.update();
        }
    }
}
