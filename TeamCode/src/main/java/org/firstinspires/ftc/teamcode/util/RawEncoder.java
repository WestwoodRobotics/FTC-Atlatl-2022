package org.firstinspires.ftc.teamcode.util;

import com.qualcomm.robotcore.hardware.DcMotorController;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

import org.firstinspires.ftc.teamcode.util.Encoder;

public final class RawEncoder implements Encoder {
    private final DcMotorEx m;
    public DcMotorSimple.Direction direction = DcMotorSimple.Direction.FORWARD;

    // TODO: add parameter for overflow
    // should overflow correction be enabled by default?
    public RawEncoder(DcMotorEx m) {
        this.m = m;
    }

    private int applyDirection(int x) {
        if (m.getDirection() == DcMotorSimple.Direction.REVERSE) {
            x = -x;
        }

        if (direction == DcMotorSimple.Direction.REVERSE) {
            x = -x;
        }

        return x;
    }

    @Override
    public PositionVelocityPair getPositionAndVelocity() {
        return new PositionVelocityPair(
                applyDirection(m.getCurrentPosition()),
                applyDirection((int) m.getVelocity())
        );
    }

    @Override
    public DcMotorController getController() {
        return m.getController();
    }
}
