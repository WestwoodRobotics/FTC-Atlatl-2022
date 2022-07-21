package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.DualNum;
import com.acmerobotics.roadrunner.Rotation2;
import com.acmerobotics.roadrunner.Time;
import com.acmerobotics.roadrunner.Twist2IncrementDual;
import com.acmerobotics.roadrunner.Vector2Dual;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.util.BNO055Wrapper;
import org.firstinspires.ftc.teamcode.util.Encoder;
import org.firstinspires.ftc.teamcode.util.Localizer;
import org.firstinspires.ftc.teamcode.util.RawEncoder;

@Config
public final class TwoDeadWheelLocalizer implements Localizer {
    public static double PAR_Y_TICKS = 0.0;
    public static double PERP_X_TICKS = 0.0;

    public final Encoder par, perp;
    public final BNO055Wrapper imu;

    private int lastParPos, lastPerpPos;
    private Rotation2 lastHeading;

    private final double inPerTick;

    public TwoDeadWheelLocalizer(HardwareMap hardwareMap, BNO055Wrapper imu, double inPerTick) {
        par = new RawEncoder(hardwareMap.get(DcMotorEx.class, "par"));
        perp = new RawEncoder(hardwareMap.get(DcMotorEx.class, "perp"));
        this.imu = imu;

        lastParPos = par.getPositionAndVelocity().position;
        lastPerpPos = perp.getPositionAndVelocity().position;
        lastHeading = imu.getHeading();

        this.inPerTick = inPerTick;
    }

    public Twist2IncrementDual<Time> updateAndGetIncr() {
        Encoder.PositionVelocityPair parPosVel = par.getPositionAndVelocity();
        Encoder.PositionVelocityPair perpPosVel = perp.getPositionAndVelocity();
        Rotation2 heading = imu.getHeading();

        int parPosDelta = parPosVel.position - lastParPos;
        int perpPosDelta = perpPosVel.position - lastPerpPos;
        double headingDelta = heading.minus(lastHeading);

        double headingVel = imu.getHeadingVelocity();

        Twist2IncrementDual<Time> twistIncr = new Twist2IncrementDual<>(
                new Vector2Dual<>(
                        new DualNum<>(new double[] {
                                inPerTick * (parPosDelta - PAR_Y_TICKS * headingDelta),
                                inPerTick * (parPosVel.velocity - PAR_Y_TICKS * headingVel),
                        }),
                        new DualNum<>(new double[] {
                                inPerTick * (perpPosDelta - PERP_X_TICKS * headingDelta),
                                inPerTick * (perpPosVel.velocity - PERP_X_TICKS * headingVel),
                        })
                ),
                new DualNum<>(new double[] {
                        headingDelta,
                        headingVel,
                })
        );

        lastParPos = parPosVel.position;
        lastPerpPos = perpPosVel.position;
        lastHeading = heading;

        return twistIncr;
    }
}
