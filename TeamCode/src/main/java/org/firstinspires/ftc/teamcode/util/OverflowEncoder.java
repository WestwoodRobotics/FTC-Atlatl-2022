package org.firstinspires.ftc.teamcode.util;

import com.qualcomm.robotcore.hardware.DcMotorController;
import com.qualcomm.robotcore.util.ElapsedTime;

public final class OverflowEncoder implements Encoder {
    // encoder velocities are sent as 16-bit ints
    // by the time they reach here, they are widened into an int and possibly negated
    private static final int CPS_STEP = 0x10000;

    private static int inverseOverflow(int input, double estimate) {
        while (Math.abs(estimate - input) > CPS_STEP / 2.0) {
            if (input < estimate) {
                input += CPS_STEP;
            } else {
                input -= CPS_STEP;
            }
        }
        return input;
    }

    public final RawEncoder encoder;

    private int lastPosition;
    private final ElapsedTime lastUpdate;

    public OverflowEncoder(RawEncoder e) {
        encoder = e;

        lastPosition = e.getPositionAndVelocity().position;
        lastUpdate = new ElapsedTime();
    }

    @Override
    public PositionVelocityPair getPositionAndVelocity() {
        PositionVelocityPair p = encoder.getPositionAndVelocity();
        double dt = lastUpdate.seconds();
        double velocityEstimate = (p.position - lastPosition) / dt;

        lastPosition = p.position;
        lastUpdate.reset();

        return new PositionVelocityPair(
                p.position,
                inverseOverflow(p.velocity, velocityEstimate)
        );
    }

    @Override
    public DcMotorController getController() {
        return encoder.getController();
    }
}
