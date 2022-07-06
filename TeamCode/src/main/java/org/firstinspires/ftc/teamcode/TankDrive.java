package org.firstinspires.ftc.teamcode;

import com.acmerobotics.roadrunner.Time;
import com.acmerobotics.roadrunner.Transform2;
import com.acmerobotics.roadrunner.Twist2;
import com.acmerobotics.roadrunner.Twist2IncrementDual;
import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.util.BNO055Wrapper;
import org.firstinspires.ftc.teamcode.util.Localizer;
import org.firstinspires.ftc.teamcode.util.LynxFirmwareVersion;

import java.util.Arrays;
import java.util.List;

public final class TankDrive {
    public final List<DcMotorEx> leftMotors, rightMotors;

    public final BNO055Wrapper imu;

    public final Localizer localizer;
    public Transform2 txRobotWorld;

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