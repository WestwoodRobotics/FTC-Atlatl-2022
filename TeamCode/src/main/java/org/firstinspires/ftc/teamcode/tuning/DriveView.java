package org.firstinspires.ftc.teamcode.tuning;

import com.qualcomm.robotcore.hardware.DcMotorController;
import com.qualcomm.robotcore.hardware.DcMotorEx;

import org.firstinspires.ftc.teamcode.util.BNO055Wrapper;
import org.firstinspires.ftc.teamcode.util.Encoder;
import org.firstinspires.ftc.teamcode.util.Localizer;
import org.firstinspires.ftc.teamcode.MecanumDrive;
import org.firstinspires.ftc.teamcode.TankDrive;
import org.firstinspires.ftc.teamcode.ThreeDeadWheelLocalizer;
import org.firstinspires.ftc.teamcode.TwoDeadWheelLocalizer;

import java.util.Arrays;
import java.util.Collections;
import java.util.List;

final class DriveView {
    public final List<DcMotorEx> leftMotors, rightMotors;
    public final List<Encoder> leftEncoders, rightEncoders, parEncoders;

    public final BNO055Wrapper imu;

    public final String type;

    private static List<Encoder> extractEncoders(Localizer l) {
    }

    public DriveView(Object d) {
        final Localizer localizer;
        if (d instanceof MecanumDrive) {
            type = "mecanum";

            MecanumDrive md = (MecanumDrive) d;
            leftMotors = Arrays.asList(md.leftFront, md.leftRear);
            rightMotors = Arrays.asList(md.rightFront, md.rightRear);
            localizer = md.localizer;
            imu = md.imu;
        } else if (d instanceof TankDrive) {
            type = "tank";

            TankDrive td = (TankDrive) d;
            leftMotors = Collections.unmodifiableList(td.leftMotors);
            rightMotors = Collections.unmodifiableList(td.rightMotors);
            localizer = td.localizer;
            imu = td.imu;
        } else {
            throw new RuntimeException();
        }

        if (localizer instanceof TwoDeadWheelLocalizer) {
            return Collections.singletonList(((TwoDeadWheelLocalizer) l).par);
        } else if (localizer instanceof ThreeDeadWheelLocalizer) {
            ThreeDeadWheelLocalizer l3 = (ThreeDeadWheelLocalizer) l;
            return Arrays.asList(l3.par0, l3.par1);
        } else {
            throw new RuntimeException();
        }

        DcMotorController c1 = parEncoders.get(0).getController();
        for (Encoder e : parEncoders) {
            DcMotorController c2 = e.getController();
            if (c1 != c2) {
                throw new IllegalArgumentException("all encoders must be attached to the same hub");
            }
        }

        // TODO: verifying the bulk cache mode is both annoying and paranoid
    }
}
