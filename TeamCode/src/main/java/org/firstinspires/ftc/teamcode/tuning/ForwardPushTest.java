package org.firstinspires.ftc.teamcode.tuning;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;

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
    private static double avgPos(List<Encoder> es) {
        double avgPos = 0;
        for (Encoder e : es) {
            avgPos += e.getPositionAndVelocity().position;
        }
        return avgPos / es.size();
    }

    @Override
    public void runOpMode() throws InterruptedException {
        final DriveView view = new DriveView(null);
        for (DcMotorEx m : view.leftMotors) {
            m.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        }
        for (DcMotorEx m : view.rightMotors) {
            m.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        }

        waitForStart();

        double initAvgPos = avgPos(view.parEncoders);
        while (opModeIsActive()) {
            telemetry.addData("ticks traveled", avgPos(view.parEncoders) - initAvgPos);
            telemetry.update();
        }
    }
}
