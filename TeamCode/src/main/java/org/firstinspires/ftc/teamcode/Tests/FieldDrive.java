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

package org.firstinspires.ftc.teamcode.Tests;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.hardware.bosch.BNO055IMU;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
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

@TeleOp(name="Field centric chassis")

public class FieldDrive extends OpMode

{
    double jTheta = 0;
    double rTheta = 0;
    double fTheta = 0;
    double rThetaRad = 0;
    double fRadTheta = 0.0;
    double mag = 0;
    double nX = 0;
    double nY = 0;
    double orgAngle;
    // Declare OpMode members.
    private ElapsedTime runtime = new ElapsedTime();
    private DcMotorEx frontLeft;
    private DcMotorEx frontRight;
    private DcMotorEx backLeft;
    private DcMotorEx backRight;
    public BNO055IMU imu;

    double leftFrontPower;
    double rightFrontPower;
    double leftBackPower;
    double rightBackPower;
    double offSetAngle = 0;
    double currentActualAngle = 0;
    double currentActualAngleRadians = 0;

    @Override
    public void init() {
        telemetry.addData("Status", "Initialized");


        frontLeft = hardwareMap.get(DcMotorEx.class, "leftFront");
        frontRight = hardwareMap.get(DcMotorEx.class, "rightFront");
        backLeft = hardwareMap.get(DcMotorEx.class, "leftBack");
        backRight = hardwareMap.get(DcMotorEx.class, "rightBack");

        frontLeft.setDirection(DcMotor.Direction.REVERSE);
        frontRight.setDirection(DcMotor.Direction.FORWARD);
        backLeft.setDirection(DcMotor.Direction.REVERSE);
        backRight.setDirection(DcMotor.Direction.FORWARD);
        //Zero Power Behavior
        frontLeft.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        frontRight.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        backLeft.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        backRight.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);

        frontLeft.setVelocityPIDFCoefficients(15, 0, 0, 0);
        frontRight.setVelocityPIDFCoefficients(15, 0, 0, 0);
        backLeft.setVelocityPIDFCoefficients(15, 0, 0, 0);
        backRight.setVelocityPIDFCoefficients(15, 0, 0, 0);


        // Tell the driver that initialization is complete.
        telemetry.addData("Status", "Initialized");

        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();

        parameters.mode                = BNO055IMU.SensorMode.IMU;
        parameters.angleUnit           = BNO055IMU.AngleUnit.DEGREES;
        parameters.accelUnit           = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        parameters.loggingEnabled      = false;

        // Retrieve and initialize the IMU. We expect the IMU to be attached to an I2C port
        // on a Core Device Interface Module and named "imu".
        imu = hardwareMap.get(BNO055IMU.class, "imu");

        imu.initialize(parameters);
        BNO055IMUUtil.remapAxes(imu, AxesOrder.XYZ, AxesSigns.NPN);
    }

    /*
     * Code to run REPEATEDLY after the driver hits INIT, but before they hit PLAY
     */
    @Override
    public void init_loop() {
    }

    /*
     * Code to run ONCE when the driver hits PLAY
     */
    @Override
    public void start() {
        runtime.reset();
    }

    /*
     * Code to run REPEATEDLY after the driver hits PLAY but before they hit STOP
     */
    @Override
    public void loop() {
        double straight = -gamepad1.left_stick_y;
        double strafing = gamepad1.left_stick_x;
        double turn = (gamepad1.right_stick_x)*0.6;

        this.calcNewXY(strafing, straight);

        leftFrontPower = (-nY - nX - turn);
        rightFrontPower = (-nY - nX + turn);
        leftBackPower = (-nY + nX - turn);
        rightBackPower = (-nY + nX + turn);

        frontLeft.setVelocity(leftFrontPower*3000);
        frontRight.setVelocity(rightBackPower*3000);
        backLeft.setVelocity(leftBackPower*3000);
        backRight.setVelocity(rightFrontPower*3000);

        telemetry.addData("newX: ", strafing);
        telemetry.addData("newY: ", straight);
        telemetry.addData("turn: ", turn);
        telemetry.addData("currentAngle: ", currentActualAngle);

        if (gamepad1.dpad_down && gamepad1.a){
            OffSet();
        }
    }

    public void calcNewXY(double x, double y) {
        orgAngle = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES).firstAngle;
        orgAngle = orgAngle + 90;
        if (orgAngle < 0) {
            orgAngle = orgAngle + 360;
        }

        if (orgAngle > 360) {
            orgAngle = orgAngle - 360;
        }
        rTheta = orgAngle + offSetAngle;
        rThetaRad = rTheta * (Math.PI / 180.0);
        double cosTheta = Math.cos(rThetaRad);
        double sinTheta = Math.sin(rThetaRad);
        nX = (x * sinTheta) - (y * cosTheta);
        nY = (x * cosTheta) + (y * sinTheta);
    }


    @Override
    public void stop() {
    }

    public double getAngle() {
        return imu.getAngularOrientation(AxesReference.INTRINSIC,AxesOrder.XYZ, AngleUnit.DEGREES).firstAngle;
    }


    public void OffSet(){
        offSetAngle = 0 - this.getAngle();
    }

}