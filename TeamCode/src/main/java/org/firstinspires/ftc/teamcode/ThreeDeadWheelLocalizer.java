package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.DualNum;
import com.acmerobotics.roadrunner.Time;
import com.acmerobotics.roadrunner.Twist2IncrementDual;
import com.acmerobotics.roadrunner.Vector2Dual;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.util.Encoder;
import org.firstinspires.ftc.teamcode.util.Localizer;
import org.firstinspires.ftc.teamcode.util.RawEncoder;

@Config
public final class ThreeDeadWheelLocalizer implements Localizer {
    // measured in ticks
    public static double PAR0_Y = 0.0;
    public static double PAR1_Y = 0.0;
    public static double PERP_X = 0.0;

    public static double IN_PER_TICK = 0.0;

    public final Encoder par0, par1, perp;

    private int lastPar0Pos, lastPar1Pos, lastPerpPos;

    public ThreeDeadWheelLocalizer(HardwareMap hardwareMap) {
        par0 = new RawEncoder(hardwareMap.get(DcMotorEx.class, "par0"));
        par1 = new RawEncoder(hardwareMap.get(DcMotorEx.class, "par1"));
        perp = new RawEncoder(hardwareMap.get(DcMotorEx.class, "perp"));

        lastPar0Pos = par0.getPositionAndVelocity().position;
        lastPar1Pos = par1.getPositionAndVelocity().position;
        lastPerpPos = perp.getPositionAndVelocity().position;
    }

    public Twist2IncrementDual<Time> updateAndGetIncr() {
        double par0PosDelta, par1PosDelta, perpPosDelta;
        double par0Vel, par1Vel, perpVel;

        {
            Encoder.PositionVelocityPair par0PosVel = par0.getPositionAndVelocity();
            Encoder.PositionVelocityPair par1PosVel = par1.getPositionAndVelocity();
            Encoder.PositionVelocityPair perpPosVel = perp.getPositionAndVelocity();

            par0PosDelta = IN_PER_TICK * (par0PosVel.position - lastPar0Pos);
            par1PosDelta = IN_PER_TICK * (par1PosVel.position - lastPar1Pos);
            perpPosDelta = IN_PER_TICK * (perpPosVel.position - lastPerpPos);

            lastPar0Pos = par0PosVel.position;
            lastPar1Pos = par1PosVel.position;
            lastPerpPos = perpPosVel.position;

            par0Vel = IN_PER_TICK * par0PosVel.velocity;
            par1Vel = IN_PER_TICK * par1PosVel.velocity;
            perpVel = IN_PER_TICK * perpPosVel.velocity;
        }

        return new Twist2IncrementDual<>(
                new Vector2Dual<>(
                        new DualNum<>(new double[] {
                                (PAR0_Y * par1PosDelta - PAR1_Y * par0PosDelta) / (PAR0_Y - PAR1_Y),
                                (PAR0_Y * par1Vel - PAR1_Y * par0Vel) / (PAR0_Y - PAR1_Y),
                        }),
                        new DualNum<>(new double[] {
                                PERP_X / (PAR0_Y - PAR1_Y) * (par1PosDelta - par0PosDelta) + perpPosDelta,
                                PERP_X / (PAR0_Y - PAR1_Y) * (par1Vel - par0Vel) + perpVel,
                        })
                ),
                new DualNum<>(new double[] {
                        (par0PosDelta - par1PosDelta) / (PAR0_Y - PAR1_Y),
                        (par0Vel - par1Vel) / (PAR0_Y - PAR1_Y),
                })
        );
    }
}
