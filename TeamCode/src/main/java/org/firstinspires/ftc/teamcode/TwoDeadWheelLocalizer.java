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
    // measured in ticks
    public static double PAR_Y = 0.0;
    public static double PERP_X = 0.0;

    public static double IN_PER_TICK = 0.0;

    public final Encoder par, perp;
    public final BNO055Wrapper imu;

    private int lastParPos, lastPerpPos;
    private Rotation2 lastHeading;

    public TwoDeadWheelLocalizer(HardwareMap hardwareMap, BNO055Wrapper imu) {
        par = new RawEncoder(hardwareMap.get(DcMotorEx.class, "par"));
        perp = new RawEncoder(hardwareMap.get(DcMotorEx.class, "perp"));
        this.imu = imu;

        lastParPos = par.getPositionAndVelocity().position;
        lastPerpPos = perp.getPositionAndVelocity().position;
        lastHeading = imu.getHeading();
    }

    public Twist2IncrementDual<Time> updateAndGetIncr() {
        double parPosDelta, perpPosDelta, headingDelta;
        double parVel, perpVel, headingVel;

        {
            Encoder.PositionVelocityPair parPosVel = par.getPositionAndVelocity();
            Encoder.PositionVelocityPair perpPosVel = perp.getPositionAndVelocity();
            Rotation2 heading = imu.getHeading();

            parPosDelta = IN_PER_TICK * (parPosVel.position - lastParPos);
            perpPosDelta = IN_PER_TICK * (perpPosVel.position - lastPerpPos);
            headingDelta = heading.minus(lastHeading);

            lastParPos = parPosVel.position;
            lastPerpPos = perpPosVel.position;
            lastHeading = heading;

            parVel = IN_PER_TICK * parPosVel.velocity;
            perpVel = IN_PER_TICK * perpPosVel.velocity;
            headingVel = imu.getHeadingVelocity();
        }

        return new Twist2IncrementDual<>(
                new Vector2Dual<>(
                        new DualNum<>(new double[] {
                                parPosDelta - PAR_Y * headingDelta,
                                parVel - PAR_Y * headingVel,
                        }),
                        new DualNum<>(new double[] {
                                perpPosDelta - PERP_X * headingDelta,
                                perpVel - PERP_X * headingVel,
                        })
                ),
                new DualNum<>(new double[] {
                        headingDelta,
                        headingVel,
                })
        );
    }
}
