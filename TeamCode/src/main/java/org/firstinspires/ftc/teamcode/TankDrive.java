package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.AccelConstraintFun;
import com.acmerobotics.roadrunner.CancelableProfile;
import com.acmerobotics.roadrunner.DualNum;
import com.acmerobotics.roadrunner.HolonomicController;
import com.acmerobotics.roadrunner.MotorFeedforward;
import com.acmerobotics.roadrunner.PosePath;
import com.acmerobotics.roadrunner.Position2;
import com.acmerobotics.roadrunner.PositionPathBuilder;
import com.acmerobotics.roadrunner.ProfileAccelConstraintFun;
import com.acmerobotics.roadrunner.Profiles;
import com.acmerobotics.roadrunner.Rotation2;
import com.acmerobotics.roadrunner.TangentPath;
import com.acmerobotics.roadrunner.TankKinematics;
import com.acmerobotics.roadrunner.Time;
import com.acmerobotics.roadrunner.TimeProfile;
import com.acmerobotics.roadrunner.Transform2;
import com.acmerobotics.roadrunner.Transform2Dual;
import com.acmerobotics.roadrunner.Twist2;
import com.acmerobotics.roadrunner.Twist2Dual;
import com.acmerobotics.roadrunner.Twist2IncrementDual;
import com.acmerobotics.roadrunner.Vector2;
import com.acmerobotics.roadrunner.VelConstraintFun;
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

@Config
public final class TankDrive {
    public static double IN_PER_TICK = 0;
    public static double TRACK_WIDTH_TICKS = 0;

    public static double kS = 0;
    public static double kV = 0;
    public static double kA = 0;

    public final TankKinematics kinematics = new TankKinematics(IN_PER_TICK * TRACK_WIDTH_TICKS);

    public final MotorFeedforward feedforward = new MotorFeedforward(kS, kV, kA);

    public final VelConstraintFun defaultVelConstraint = kinematics.new WheelVelConstraintFun(50);
    public final AccelConstraintFun defaultAccelConstraint = new ProfileAccelConstraintFun(-30, 50);

    public final List<DcMotorEx> leftMotors, rightMotors;

    public final BNO055Wrapper imu;

    public final VoltageSensor voltageSensor;

    public final Localizer localizer = new DriveLocalizer();
    public Transform2 pose = new Transform2(new Vector2(0, 0), Rotation2.exp(0));

    public final double inPerTick = IN_PER_TICK;

    public class DriveLocalizer implements Localizer {
        public final List<Encoder> leftEncs, rightEncs;

        private double lastLeftPos, lastRightPos;

        public DriveLocalizer() {
            {
                List<Encoder> leftEncs = new ArrayList<>();
                for (DcMotorEx m : leftMotors) {
                    Encoder e = new RawEncoder(m);
                    leftEncs.add(e);
                    lastLeftPos += e.getPositionAndVelocity().position;
                }
                lastLeftPos /= leftEncs.size();
                this.leftEncs = Collections.unmodifiableList(leftEncs);
            }

            {
                List<Encoder> rightEncs = new ArrayList<>();
                for (DcMotorEx m : rightMotors) {
                    Encoder e = new RawEncoder(m);
                    rightEncs.add(e);
                    lastRightPos += e.getPositionAndVelocity().position;
                }
                lastRightPos /= rightEncs.size();
                this.rightEncs = Collections.unmodifiableList(rightEncs);
            }
        }

        @Override
        public Twist2IncrementDual<Time> updateAndGetIncr() {
            double meanLeftPos = 0.0, meanLeftVel = 0.0;
            for (Encoder e : leftEncs) {
                Encoder.PositionVelocityPair p = e.getPositionAndVelocity();
                meanLeftPos += p.position;
                meanLeftVel += p.velocity;
            }
            meanLeftPos /= leftEncs.size();
            meanLeftVel /= leftEncs.size();

            double meanRightPos = 0.0, meanRightVel = 0.0;
            for (Encoder e : rightEncs) {
                Encoder.PositionVelocityPair p = e.getPositionAndVelocity();
                meanRightPos += p.position;
                meanRightVel += p.velocity;
            }
            meanRightPos /= rightEncs.size();
            meanRightVel /= rightEncs.size();

            TankKinematics.WheelIncrements<Time> incrs = new TankKinematics.WheelIncrements<>(
                    new DualNum<Time>(new double[] {
                            meanLeftPos - lastLeftPos,
                            meanLeftVel
                    }).times(inPerTick),
                    new DualNum<Time>(new double[] {
                            meanRightPos - lastRightPos,
                            meanRightVel,
                    }).times(inPerTick)
            );

            lastLeftPos = meanLeftPos;
            lastRightPos = meanRightPos;

            return kinematics.forward(incrs);
        }
    }

    public TankDrive(HardwareMap hardwareMap) {
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
    }

    public final class FollowTrajectoryAction implements Action {
        public final PosePath path;

        private final CancelableProfile cancelableProfile;
        private TimeProfile profile;
        private double beginTs, beginDisp;

        public FollowTrajectoryAction(PosePath path, CancelableProfile profile) {
            this.path = path;
            cancelableProfile = profile;
            this.profile = new TimeProfile(cancelableProfile.baseProfile);
        }

        @Override
        public void init() {
            beginTs = clock();
        }

        @Override
        public boolean loop() {
            double t = clock() - beginTs;
            if (t >= profile.duration) {
                for (DcMotorEx m : leftMotors) {
                    m.setPower(0);
                }
                for (DcMotorEx m : rightMotors) {
                    m.setPower(0);
                }

                return false;
            }

            DualNum<Time> x = profile.get(t);
            Transform2Dual<Time> txWorldTarget = path.get(beginDisp + x.value(), 3).reparam(x);

            Twist2 robotVelRobot = updatePoseEstimateAndGetActualVel();

            Twist2Dual<Time> command = new HolonomicController(0, 0, 0, 0, 0, 0)
                    .compute(txWorldTarget, pose, robotVelRobot);

            TankKinematics.WheelVelocities<Time> wheelVels = kinematics.inverse(command);
            double voltage = voltageSensor.getVoltage();
            for (DcMotorEx m : leftMotors) {
                m.setPower(feedforward.compute(wheelVels.left) / voltage);
            }
            for (DcMotorEx m : rightMotors) {
                m.setPower(feedforward.compute(wheelVels.right) / voltage);
            }

            return true;
        }

        public void cancel() {
            if (beginDisp != 0) {
                throw new IllegalStateException();
            }

            double t = clock() - beginTs;
            beginDisp = profile.get(t).get(0);
            beginTs += t;
            profile = new TimeProfile(cancelableProfile.cancel(beginDisp));
        }
    }

    public Twist2 updatePoseEstimateAndGetActualVel() {
        Twist2IncrementDual<Time> incr = localizer.updateAndGetIncr();
        pose = pose.plus(incr.value());
        return incr.velocity().value();
    }

    public static class PathBuilder {
        private final PositionPathBuilder posPathBuilder;
        private final double offset;

        private PathBuilder(PositionPathBuilder posPathBuilder, double tanOffset) {
            this.posPathBuilder = posPathBuilder;
            this.offset = tanOffset;
        }

        public PathBuilder(Position2 beginPos, Rotation2 beginTangent, boolean reversed, double eps) {
            this(new PositionPathBuilder(beginPos, beginTangent, eps), reversed ? Math.PI : 0);
        }
        public PathBuilder(Position2 beginPos, double beginTangent, boolean reversed, double eps) {
            this(beginPos, Rotation2.exp(beginTangent), reversed, eps);
        }
        public PathBuilder(Position2 beginPos, Rotation2 beginTangent, double eps) {
            this(beginPos, beginTangent, false, eps);
        }
        public PathBuilder(Position2 beginPos, double beginTangent, double eps) {
            this(beginPos, Rotation2.exp(beginTangent), eps);
        }

        public PathBuilder forward(double dist) {
            return new PathBuilder(posPathBuilder.forward(dist), offset);
        }

        public PathBuilder lineToX(double posX) {
            return new PathBuilder(posPathBuilder.lineToX(posX), offset);
        }

        public PathBuilder lineToY(double posY) {
            return new PathBuilder(posPathBuilder.lineToY(posY), offset);
        }

        public PathBuilder splineTo(Position2 pos, Rotation2 tangent) {
            return new PathBuilder(posPathBuilder.splineTo(pos, tangent), offset);
        }
        public PathBuilder splineTo(Position2 pos, double tangent) {
            return splineTo(pos, Rotation2.exp(tangent));
        }

        public TangentPath build() {
            return new TangentPath(posPathBuilder.build(), offset);
        }
    }

    public PathBuilder pathBuilder(Position2 beginPos, Rotation2 beginTangent, boolean reversed) {
        return new PathBuilder(beginPos, beginTangent, reversed, 1e-6);
    }
    public PathBuilder pathBuilder(Position2 beginPos, double beginTangent, boolean reversed) {
        return new PathBuilder(beginPos, Rotation2.exp(beginTangent), reversed, 1e-6);
    }
    public PathBuilder pathBuilder(Position2 beginPos, Rotation2 beginTangent) {
        return pathBuilder(beginPos, beginTangent, false);
    }
    public PathBuilder pathBuilder(Position2 beginPos, double beginTangent) {
        return pathBuilder(beginPos, Rotation2.exp(beginTangent), false);
    }

    public FollowTrajectoryAction followPath(PosePath path, VelConstraintFun vf, AccelConstraintFun af) {
        return new FollowTrajectoryAction(path,
                Profiles.profile(path, 0, vf, af, 0.25));
    }
    public FollowTrajectoryAction followPath(PosePath path) {
        return followPath(path, defaultVelConstraint, defaultAccelConstraint);
    }
}