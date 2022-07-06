package org.firstinspires.ftc.teamcode;

import com.acmerobotics.roadrunner.DualNum;
import com.acmerobotics.roadrunner.MecanumKinematics;
import com.acmerobotics.roadrunner.Time;
import com.acmerobotics.roadrunner.Transform2;
import com.acmerobotics.roadrunner.Twist2;
import com.acmerobotics.roadrunner.Twist2IncrementDual;
import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.VoltageSensor;

import org.firstinspires.ftc.teamcode.util.BNO055Wrapper;
import org.firstinspires.ftc.teamcode.util.Encoder;
import org.firstinspires.ftc.teamcode.util.Localizer;
import org.firstinspires.ftc.teamcode.util.LynxFirmwareVersion;
import org.firstinspires.ftc.teamcode.util.RawEncoder;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.Collections;
import java.util.List;

public final class TankDrive {
    public final List<DcMotorEx> leftMotors, rightMotors;

    public final BNO055Wrapper imu;

    public final VoltageSensor voltageSensor;

    public final Localizer localizer;
    public Transform2 txRobotWorld;

    public class DriveLocalizer implements Localizer {
        public final List<Encoder> leftEncs, rightEncs;

        private final List<Integer> lastLeftPosList = new ArrayList<>(), lastRightPosList = new ArrayList<>();

        public DriveLocalizer() {
            List<Encoder> leftEncs = new ArrayList<>(), rightEncs = new ArrayList<>();
            for (DcMotorEx m : leftMotors) {
                Encoder e = new RawEncoder(m);
                leftEncs.add(e);
                lastLeftPosList.add(e.getPositionAndVelocity().position);
            }
            for (DcMotorEx m : rightMotors) {
                Encoder e = new RawEncoder(m);
                rightEncs.add(e);
                lastRightPosList.add(e.getPositionAndVelocity().position);
            }

            this.leftEncs = Collections.unmodifiableList(leftEncs);
            this.rightEncs = Collections.unmodifiableList(rightEncs);
        }

        // TODO: finish
        @Override
        public Twist2IncrementDual<Time> updateAndGetIncr() {
            Encoder.PositionVelocityPair leftFrontPosVel = leftFront.getPositionAndVelocity();
            Encoder.PositionVelocityPair leftRearPosVel = leftRear.getPositionAndVelocity();
            Encoder.PositionVelocityPair rightRearPosVel = rightRear.getPositionAndVelocity();
            Encoder.PositionVelocityPair rightFrontPosVel = rightFront.getPositionAndVelocity();

            MecanumKinematics.WheelIncrements<Time> incrs = new MecanumKinematics.WheelIncrements<>(
                    new DualNum<>(new double[] {
                            IN_PER_TICK * (leftFrontPosVel.position - lastLeftFrontPos),
                            IN_PER_TICK * leftFrontPosVel.velocity,
                    }),
                    new DualNum<>(new double[] {
                            IN_PER_TICK * (leftRearPosVel.position - lastLeftRearPos),
                            IN_PER_TICK * leftRearPosVel.velocity,
                    }),
                    new DualNum<>(new double[] {
                            IN_PER_TICK * (rightRearPosVel.position - lastRightRearPos),
                            IN_PER_TICK * rightRearPosVel.velocity,
                    }),
                    new DualNum<>(new double[] {
                            IN_PER_TICK * (rightFrontPosVel.position - lastRightFrontPos),
                            IN_PER_TICK * rightFrontPosVel.velocity,
                    })
            );

            lastLeftFrontPos = leftFrontPosVel.position;
            lastLeftRearPos = leftRearPosVel.position;
            lastRightRearPos = rightRearPosVel.position;
            lastRightFrontPos = rightFrontPosVel.position;

            return kinematics.forward(incrs);
        }
    }

    public TankDrive(HardwareMap hardwareMap, Transform2 txRobotWorld) {
        LynxFirmwareVersion.throwIfAnyModulesBelowVersion(hardwareMap,
                new LynxFirmwareVersion(1, 8, 2));

        for (LynxModule module : hardwareMap.getAll(LynxModule.class)) {
            module.setBulkCachingMode(LynxModule.BulkCachingMode.AUTO);
        }

        leftMotors = Arrays.asList(hardwareMap.get(DcMotorEx.class, "left"));
        rightMotors = Arrays.asList(hardwareMap.get(DcMotorEx.class, "right"));

        for (DcMotorEx m : leftMotors) {
            m.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        }
        for (DcMotorEx m : rightMotors) {
            m.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        }

        BNO055IMU baseImu = hardwareMap.get(BNO055IMU.class, "imu");
        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.angleUnit = BNO055IMU.AngleUnit.RADIANS;
        baseImu.initialize(parameters);
        imu = new BNO055Wrapper(baseImu);

        voltageSensor = hardwareMap.voltageSensor.iterator().next();

        // TODO: change motor encoder directions

        localizer = new ThreeDeadWheelLocalizer(hardwareMap);
        this.txRobotWorld = txRobotWorld;
    }

    public Twist2 updatePoseEstimateAndGetActualVel() {
        Twist2IncrementDual<Time> incr = localizer.updateAndGetIncr();
        txRobotWorld = txRobotWorld.plus(incr.value());
        return incr.velocity().value();
    }
}