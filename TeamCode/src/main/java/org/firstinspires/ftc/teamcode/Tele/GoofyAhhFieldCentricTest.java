/* Copyright (c) 2017 FIRST. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted (subject to the limitations in the disclaimer below) provided that
 * the following conditions are met:
 *
 * Redistributions of source code must retain the above copyright notice, this list
 * of conditions and the following disclaimer.
 *
 * Redistributions in binary form must reproduce the above copyright notice, this
 * list of conditions and the following disclaimer in the documentation and/or
 * other materials provided with the distribution.
 *
 * Neither the name of FIRST nor the names of its contributors may be used to endorse or
 * promote products derived from this software without specific prior written permission.
 *
 * NO EXPRESS OR IMPLIED LICENSES TO ANY PARTY'S PATENT RIGHTS ARE GRANTED BY THIS
 * LICENSE. THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
 * THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

package org.firstinspires.ftc.teamcode.Tele;

import com.qualcomm.hardware.bosch.JustLoggingAccelerationIntegrator;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.Range;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.hardware.bosch.BNO055IMU;

import org.firstinspires.ftc.robotcore.external.navigation.Acceleration;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.robotcore.external.navigation.Position;
import org.firstinspires.ftc.robotcore.external.navigation.Velocity;
import org.firstinspires.ftc.teamcode.util.AxesSigns;
import org.firstinspires.ftc.teamcode.util.BNO055IMUUtil;

import java.lang.Math;


/**
 * This file contains an example of an iterative (Non-Linear) "OpMode".
 * An OpMode is a 'program' that runs in either the autonomous or the teleop period of an FTC match.
 * The names of OpModes appear on the menu of the FTC Driver Station.
 * When an selection is made from the menu, the corresponding OpMode
 * class is instantiated on the Robot Controller and executed.
 *
 * This particular OpMode just executes a basic Tank Drive Teleop for a two wheeled robot
 * It includes all the skeletal structure that all iterative OpModes contain.
 *
 * Use Android Studios to Copy this Class, and Paste it into your team's code folder with a new name.
 * Remove or comment out the @Disabled line to add this opmode to the Driver Station OpMode list
 */

@TeleOp(name="NOTSTOLENFiledCentricDriveTest", group="Iterative Opmode")

public class GoofyAhhFieldCentricTest extends OpMode

{
    // Declare OpMode members.
    private ElapsedTime runtime = new ElapsedTime();
    private DcMotorEx leftFront = null;
    private DcMotorEx rightFront = null;
    private DcMotorEx leftBack = null;
    private DcMotorEx rightBack = null;
    public BNO055IMU imu;

    double leftFrontPower;
    double rightFrontPower;
    double leftBackPower;
    double rightBackPower;
    double offSetAngle;
    double currentActualAngle = 0;
    double currentActualAngleRadians = 0;
    double theta;

    /*
     * Code to run ONCE when the driver hits INIT
     */
    @Override
    public void init() {
        telemetry.addData("Status", "Initialized");

        leftFront  = hardwareMap.get(DcMotorEx.class, "leftFront");
        rightFront = hardwareMap.get(DcMotorEx.class, "rightFront");
        leftBack  = hardwareMap.get(DcMotorEx.class, "leftBack");
        rightBack = hardwareMap.get(DcMotorEx.class, "rightBack");

        rightBack.setDirection(DcMotorEx.Direction.REVERSE);
        leftBack.setDirection(DcMotorEx.Direction.REVERSE);
        rightFront.setDirection(DcMotorEx.Direction.REVERSE);
        leftFront.setDirection(DcMotorEx.Direction.REVERSE);

        //tuning PIDs
        {
            leftFront.setVelocityPIDFCoefficients(15, 0, 0, 0);
            rightFront.setVelocityPIDFCoefficients(15, 0, 0, 0);
            leftBack.setVelocityPIDFCoefficients(15, 0, 0, 0);
            rightBack.setVelocityPIDFCoefficients(15, 0, 0, 0);
        }

        // Tell the driver that initialization is complete.
        telemetry.addData("Status", "Initialized");

        //Setting Zero Power behaviour
        leftFront.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        rightFront.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        leftBack.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        rightBack.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);

        //centric
        {
            BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();

            parameters.mode = BNO055IMU.SensorMode.IMU;
            parameters.angleUnit = BNO055IMU.AngleUnit.DEGREES;
            parameters.accelUnit = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
            parameters.loggingEnabled = false;

            // Retrieve and initialize the IMU. We expect the IMU to be attached to an I2C port
            // on a Core Device Interface Module and named "imu".
            imu = hardwareMap.get(BNO055IMU.class, "imu");

            imu.initialize(parameters);
            BNO055IMUUtil.remapAxes(imu, AxesOrder.ZXY, AxesSigns.NPN);
        }
    }
    @Override
    public void start() {
        runtime.reset();
    }

    /*
     * Code to run REPEATEDLY after the driver hits PLAY but before they hit STOP
     */
    @Override
    public void loop() {
        //centric drive
        {
            this.calculateCurAngle();

            double strafe;
            double drive;
            double turn;

            strafe = this.getNewXY(gamepad1.right_stick_x, gamepad1.right_stick_y, "X");
            drive = this.getNewXY(gamepad1.right_stick_x, gamepad1.right_stick_y, "Y");
            turn = gamepad1.left_stick_x;


            //edit equations E
            leftFrontPower = drive + strafe + turn;
            leftBackPower = drive + strafe - turn;
            rightFrontPower = drive - strafe - turn;
            rightBackPower = drive + strafe - turn;


            //edit constants
            leftFront.setVelocity(leftFrontPower * 2000);
            rightBack.setVelocity(rightBackPower * 2000);
            leftBack.setVelocity(leftBackPower * 2000);
            rightFront.setVelocity(rightFrontPower * 2000);


            //telemetry
            telemetry.addData("IMU: ", getAngle());
            telemetry.addData("zxy axis:", imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZXY, AngleUnit.DEGREES).firstAngle);
            telemetry.addData("xzx axis:", imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.XZX, AngleUnit.DEGREES).firstAngle);
            telemetry.addData("yxy axis:", imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.YXY, AngleUnit.DEGREES).firstAngle);
            telemetry.addData("strafe: ", strafe);
            telemetry.addData("drive: ", drive);
            telemetry.addData("turn: ", turn);
            telemetry.addData("currentAngle: ", currentActualAngle);
        }

    }

    //funny functions for goofy ahh  centric
    public double getAngle() {
        return imu.getAngularOrientation(AxesReference.INTRINSIC,AxesOrder.ZXY, AngleUnit.RADIANS).firstAngle;
    }

    public double getDegreesAngle() {
        return imu.getAngularOrientation(AxesReference.INTRINSIC,AxesOrder.ZXY, AngleUnit.DEGREES).firstAngle;
    }

    public double getRawExternalHeading() {
        return imu.getAngularOrientation().firstAngle;
    }

    public double getNewXY(double gamePadX, double gamePadY, String xy) {
        double newX = (gamePadX - 0)*Math.cos(currentActualAngleRadians)-(gamePadY-0)*Math.sin(currentActualAngleRadians)+0;
        double newY = (gamePadX - 0)*Math.sin(currentActualAngleRadians)+(gamePadY-0)*Math.cos(currentActualAngleRadians)+0;

        if (xy.equals("X")) {
            return newX;
        }
        else if (xy.equals("Y")) {
            return newY;
        }
        else {
            return newY;
        }
    }

    public void calculateCurAngle() {
        double imuAngle = this.getDegreesAngle();
//        currentAngle = imuAngle < 0? 360 + imuAngle: imuAngle;
        if (imuAngle < 0) {
            currentActualAngle = 360 + imuAngle;
        }
        else {
            currentActualAngle = imuAngle;
        }

        currentActualAngleRadians = currentActualAngle*(Math.PI/180);
    }

}
