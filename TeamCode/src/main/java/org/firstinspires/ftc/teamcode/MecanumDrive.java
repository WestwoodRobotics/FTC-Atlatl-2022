package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.canvas.Canvas;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.AccelConstraintFun;
import com.acmerobotics.roadrunner.AngularVelConstraintFun;
import com.acmerobotics.roadrunner.Arclength;
import com.acmerobotics.roadrunner.CancelableProfile;
import com.acmerobotics.roadrunner.DualNum;
import com.acmerobotics.roadrunner.HolonomicController;
import com.acmerobotics.roadrunner.MecanumKinematics;
import com.acmerobotics.roadrunner.MinSimpleVelConstraintFun;
import com.acmerobotics.roadrunner.MotorFeedforward;
import com.acmerobotics.roadrunner.PosePath;
import com.acmerobotics.roadrunner.Position2;
import com.acmerobotics.roadrunner.PositionPath;
import com.acmerobotics.roadrunner.PositionPathBuilder;
import com.acmerobotics.roadrunner.ProfileAccelConstraintFun;
import com.acmerobotics.roadrunner.Profiles;
import com.acmerobotics.roadrunner.Rotation2;
import com.acmerobotics.roadrunner.Rotation2Dual;
import com.acmerobotics.roadrunner.SafePathBuilder;
import com.acmerobotics.roadrunner.SafePosePathBuilder;
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

import java.util.Arrays;
import java.util.LinkedList;
import java.util.List;

@Config
public final class MecanumDrive {
    // drive model parameters
    public static double FORWARD_IN_PER_TICK = 0;
    public static double LATERAL_IN_PER_TICK = 1;
    public static double TRACK_WIDTH_TICKS = 0;

    // feedforward parameters
    public static double kS = 0;
    public static double kV = 0;
    public static double kA = 0;

    // path profile parameters
    public static double MAX_WHEEL_VEL = 50;
    public static double MIN_PROFILE_ACCEL = -30;
    public static double MAX_PROFILE_ACCEL = 50;

    // turn profile parameters
    public static double MAX_ANG_VEL = Math.PI; // shared with path
    public static double MAX_ANG_ACCEL = Math.PI;

    // path controller gains
    public static double AXIAL_GAIN = 0.0;
    public static double LATERAL_GAIN = 0.0;
    public static double HEADING_GAIN = 0.0; // shared with turn

    public static double AXIAL_VEL_GAIN = 0.0;
    public static double LATERAL_VEL_GAIN = 0.0;
    public static double HEADING_VEL_GAIN = 0.0; // shared with turn

    public final MecanumKinematics kinematics = new MecanumKinematics(
            FORWARD_IN_PER_TICK * TRACK_WIDTH_TICKS,
            FORWARD_IN_PER_TICK / LATERAL_IN_PER_TICK);

    public final MotorFeedforward feedforward = new MotorFeedforward(kS, kV, kA);

    public final VelConstraintFun defaultVelConstraint = new VelConstraintFun.Adapter(
            new MinSimpleVelConstraintFun(Arrays.asList(
                    kinematics.new WheelVelConstraintFun(MAX_WHEEL_VEL),
                    new AngularVelConstraintFun(MAX_ANG_VEL)
            )));
    public final AccelConstraintFun defaultAccelConstraint = new AccelConstraintFun.Adapter(
            new ProfileAccelConstraintFun(MIN_PROFILE_ACCEL, MAX_PROFILE_ACCEL));

    public final DcMotorEx leftFront, leftBack, rightBack, rightFront;

    public final VoltageSensor voltageSensor;

    public final BNO055Wrapper imu;

    public final Localizer localizer = new DriveLocalizer();
    public Transform2 pose = new Transform2(0, 0, 0);

    public final double inPerTick = FORWARD_IN_PER_TICK;

    private final LinkedList<Transform2> poseHistory = new LinkedList<>();

    public class DriveLocalizer implements Localizer {
        public final Encoder leftFront, leftRear, rightRear, rightFront;

        private int lastLeftFrontPos, lastLeftRearPos, lastRightRearPos, lastRightFrontPos;

        public DriveLocalizer() {
            leftFront = new RawEncoder(MecanumDrive.this.leftFront);
            leftRear = new RawEncoder(MecanumDrive.this.leftBack);
            rightRear = new RawEncoder(MecanumDrive.this.rightBack);
            rightFront = new RawEncoder(MecanumDrive.this.rightFront);

            lastLeftFrontPos = leftFront.getPositionAndVelocity().position;
            lastLeftRearPos = leftRear.getPositionAndVelocity().position;
            lastRightRearPos = rightRear.getPositionAndVelocity().position;
            lastRightFrontPos = rightFront.getPositionAndVelocity().position;
        }

        @Override
        public Twist2IncrementDual<Time> updateAndGetIncr() {
            Encoder.PositionVelocityPair leftFrontPosVel = leftFront.getPositionAndVelocity();
            Encoder.PositionVelocityPair leftRearPosVel = leftRear.getPositionAndVelocity();
            Encoder.PositionVelocityPair rightRearPosVel = rightRear.getPositionAndVelocity();
            Encoder.PositionVelocityPair rightFrontPosVel = rightFront.getPositionAndVelocity();

            MecanumKinematics.WheelIncrements<Time> incrs = new MecanumKinematics.WheelIncrements<>(
                    new DualNum<Time>(new double[] {
                            leftFrontPosVel.position - lastLeftFrontPos,
                            leftFrontPosVel.velocity,
                    }).times(inPerTick),
                    new DualNum<Time>(new double[] {
                            leftRearPosVel.position - lastLeftRearPos,
                            leftRearPosVel.velocity,
                    }).times(inPerTick),
                    new DualNum<Time>(new double[] {
                            rightRearPosVel.position - lastRightRearPos,
                            rightRearPosVel.velocity,
                    }).times(inPerTick),
                    new DualNum<Time>(new double[] {
                            rightFrontPosVel.position - lastRightFrontPos,
                            rightFrontPosVel.velocity,
                    }).times(inPerTick)
            );

            lastLeftFrontPos = leftFrontPosVel.position;
            lastLeftRearPos = leftRearPosVel.position;
            lastRightRearPos = rightRearPosVel.position;
            lastRightFrontPos = rightFrontPosVel.position;

            return kinematics.forward(incrs);
        }
    }

    public MecanumDrive(HardwareMap hardwareMap) {
        LynxFirmwareVersion.throwIfAnyModulesBelowVersion(hardwareMap,
                new LynxFirmwareVersion(1, 8, 2));

        for (LynxModule module : hardwareMap.getAll(LynxModule.class)) {
            module.setBulkCachingMode(LynxModule.BulkCachingMode.AUTO);
        }

        leftFront = hardwareMap.get(DcMotorEx.class, "leftFront");
        leftBack = hardwareMap.get(DcMotorEx.class, "leftBack");
        rightBack = hardwareMap.get(DcMotorEx.class, "rightBack");
        rightFront = hardwareMap.get(DcMotorEx.class, "rightFront");

        leftFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        leftBack.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightBack.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

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

        private boolean active;
        private final double[] xPoints, yPoints;

        public FollowTrajectoryAction(PosePath path, CancelableProfile profile) {
            this.path = path;
            cancelableProfile = profile;
            this.profile = new TimeProfile(cancelableProfile.baseProfile);

            List<Double> disps = com.acmerobotics.roadrunner.Math.range(
                    0, path.length(), (int) Math.ceil(path.length() / 2));
            xPoints = new double[disps.size()];
            yPoints = new double[disps.size()];
            for (int i = 0; i < disps.size(); i++) {
                Transform2 t = path.get(disps.get(i), 1).value();
                xPoints[i] = t.trans.x;
                yPoints[i] = t.trans.y;
            }
        }

        @Override
        public void init() {
            beginTs = clock();

            active = true;
        }

        @Override
        public boolean loop(TelemetryPacket p) {
            double t = clock() - beginTs;
            if (t >= profile.duration) {
                leftFront.setPower(0);
                leftBack.setPower(0);
                rightBack.setPower(0);
                rightFront.setPower(0);

                active = false;

                return false;
            }

            DualNum<Time> x = profile.get(t);
            Transform2Dual<Time> txWorldTarget = path.get(beginDisp + x.value(), 3).reparam(x);

            Twist2 robotVelRobot = updatePoseEstimateAndGetActualVel();

            Twist2Dual<Time> command = new HolonomicController(
                    AXIAL_GAIN, LATERAL_GAIN, HEADING_GAIN,
                    AXIAL_VEL_GAIN, LATERAL_VEL_GAIN, HEADING_VEL_GAIN
            )
                    .compute(txWorldTarget, pose, robotVelRobot);

            MecanumKinematics.WheelVelocities<Time> wheelVels = kinematics.inverse(command);
            double voltage = voltageSensor.getVoltage();
            leftFront.setPower(feedforward.compute(wheelVels.leftFront) / voltage);
            leftBack.setPower(feedforward.compute(wheelVels.leftBack) / voltage);
            rightBack.setPower(feedforward.compute(wheelVels.rightBack) / voltage);
            rightFront.setPower(feedforward.compute(wheelVels.rightFront) / voltage);

            p.put("x", pose.trans.x);
            p.put("y", pose.trans.y);
            p.put("heading (deg)", Math.toDegrees(pose.rot.log()));

            // TODO: dedupe with controller compute()?
            Transform2 error = txWorldTarget.value().inverse().times(pose);
            p.put("xError", error.trans.x);
            p.put("yError", error.trans.y);
            p.put("headingError (deg)", Math.toDegrees(error.rot.log()));

            // only draw when active; only one drive action should be active at a time
            Canvas c = p.fieldOverlay();

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

        @Override
        public void draw(Canvas c) {
            c.setStrokeWidth(1);
            c.setStroke(active ? "#4CAF50FF" : "#4CAF507A");
            c.strokePolyline(xPoints, yPoints);
        }
    }

    public final class TurnAction implements Action {
        private final Transform2 beginPose;

        private final TimeProfile profile;
        private double beginTs;

        private boolean active;

        public TurnAction(Transform2 beginPose, double rot) {
            this.beginPose = beginPose;
            profile = new TimeProfile(Profiles.constantProfile(rot, 0, MAX_ANG_VEL, -MAX_ANG_ACCEL, MAX_ANG_ACCEL).baseProfile);
        }

        public TurnAction(Transform2 startPose, Rotation2 rot) {
            this(startPose, rot.log());
        }

        @Override
        public void init() {
            beginTs = clock();

            active = true;
        }

        @Override
        public boolean loop(TelemetryPacket p) {
            double t = clock() - beginTs;
            if (t >= profile.duration) {
                leftFront.setPower(0);
                leftBack.setPower(0);
                rightBack.setPower(0);
                rightFront.setPower(0);

                active = false;

                return false;
            }

            DualNum<Time> x = profile.get(t);
            Transform2Dual<Time> txWorldTarget = Rotation2Dual.exp(x).times(beginPose);

            Twist2 robotVelRobot = updatePoseEstimateAndGetActualVel();

            Twist2Dual<Time> command = new HolonomicController(
                    AXIAL_GAIN, LATERAL_GAIN, HEADING_GAIN,
                    AXIAL_VEL_GAIN, LATERAL_VEL_GAIN, HEADING_VEL_GAIN
            )
                    .compute(txWorldTarget, pose, robotVelRobot);

            MecanumKinematics.WheelVelocities<Time> wheelVels = kinematics.inverse(command);
            double voltage = voltageSensor.getVoltage();
            leftFront.setPower(feedforward.compute(wheelVels.leftFront) / voltage);
            leftBack.setPower(feedforward.compute(wheelVels.leftBack) / voltage);
            rightBack.setPower(feedforward.compute(wheelVels.rightBack) / voltage);
            rightFront.setPower(feedforward.compute(wheelVels.rightFront) / voltage);

            Canvas c = p.fieldOverlay();
            drawPoseHistory(c);

            c.setStroke("#4CAF50");
            drawRobot(c, txWorldTarget.value());

            c.setStroke("#3F51B5");
            drawRobot(c, pose);

            return true;
        }

        @Override
        public void draw(Canvas c) {
            c.setFill(active ? "#7C4DFFFF" : "#7C4DFF7A");
            c.fillCircle(beginPose.trans.x, beginPose.trans.y, 2);
        }
    }

    private Twist2 updatePoseEstimateAndGetActualVel() {
        Twist2IncrementDual<Time> incr = localizer.updateAndGetIncr();
        pose = pose.plus(incr.value());

        poseHistory.add(pose);
        while (poseHistory.size() > 100) {
            poseHistory.removeFirst();
        }

        return incr.velocity().value();
    }

    private void drawPoseHistory(Canvas c) {
        double[] xPoints = new double[poseHistory.size()];
        double[] yPoints = new double[poseHistory.size()];

        int i = 0;
        for (Transform2 t : poseHistory) {
            xPoints[i] = t.trans.x;
            yPoints[i] = t.trans.y;

            i++;
        }

        c.setStrokeWidth(1);
        c.setStroke("#3F51B5");
        c.strokePolyline(xPoints, yPoints);
    }

    private static void drawRobot(Canvas c, Transform2 t) {
        final double ROBOT_RADIUS = 9;

        c.setStrokeWidth(1);
        c.strokeCircle(t.trans.x, t.trans.y, ROBOT_RADIUS);

        Vector2 halfv = t.rot.vec().times(0.5 * ROBOT_RADIUS);
        Position2 p1 = t.trans.bind().plus(halfv);
        Position2 p2 = p1.plus(halfv);
        c.strokeLine(p1.x, p1.y, p2.x, p2.y);
    }

    public PositionPathBuilder posPathBuilder(Position2 beginPos, Rotation2 beginTangent) {
        return new PositionPathBuilder(beginPos, beginTangent, 1e-6);
    }
    public PositionPathBuilder posPathBuilder(Position2 beginPos, double beginTangent) {
        return new PositionPathBuilder(beginPos, beginTangent, 1e-6);
    }

    public SafePosePathBuilder posePathBuilder(PositionPath<Arclength> path, Rotation2 beginHeading) {
        return new SafePosePathBuilder(path, beginHeading);
    }
    public SafePosePathBuilder posePathBuilder(PositionPath<Arclength> path, double beginHeading) {
        return new SafePosePathBuilder(path, beginHeading);
    }

    public SafePathBuilder pathBuilder(Transform2 beginPose, Rotation2 beginTangent) {
        return new SafePathBuilder(beginPose, beginTangent, 1e-6);
    }
    public SafePathBuilder pathBuilder(Transform2 beginPose, double beginTangent) {
        return new SafePathBuilder(beginPose, beginTangent, 1e-6);
    }

    public Action.BaseBuilder actionBuilder() {
        return new Action.BaseBuilder();
    }

    public FollowTrajectoryAction followPath(PosePath path, VelConstraintFun vf, AccelConstraintFun af) {
        return new FollowTrajectoryAction(path,
                Profiles.profile(path, 0, vf, af, 0.25));
    }
    public FollowTrajectoryAction followPath(PosePath path) {
        return followPath(path, defaultVelConstraint, defaultAccelConstraint);
    }
}