package org.firstinspires.ftc.teamcode.Tests;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;


@TeleOp(name = "lift power test")
public class LiftPoweredTest extends OpMode {
    public DcMotorEx lift = null;
    public int liftPos = 0;
    public int liftTarget = 0;

    @Override
    public void init() {
        lift = hardwareMap.get(DcMotorEx.class, "lift");
        lift.setDirection(DcMotorEx.Direction.REVERSE);
        lift.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        lift.setTargetPosition(liftTarget);
        lift.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);


        //lift
        {
            liftPos = lift.getCurrentPosition();
            //manual
            if (!(gamepad2.right_trigger - gamepad2.left_trigger == 0)) {
                liftTarget += Math.round(gamepad2.right_trigger - gamepad2.left_trigger) * 25;
            } else {
                //auto
                if (gamepad2.a) {
                    liftTarget = 0;
                } else if (gamepad2.b) {
                    liftTarget = 1300;
                } else if (gamepad2.x) {
                    liftTarget = 2200;
                } else if (gamepad2.y) {
                    liftTarget = 3000;
                }
            }

            //limits
            if (liftTarget > 3000) {
                liftTarget = 3000;
            } else if (liftTarget < 0) {
                liftTarget = 0;
            }

            telemetry.addData("lift position", lift.getCurrentPosition());

            //power setting
            lift.setTargetPosition(liftTarget);

            if (liftTarget > liftPos) {
                lift.setPower(1);
            } else if (liftPos > liftTarget) {
                lift.setPower(-1);
            } else if (liftPos == liftTarget) {
                lift.setPower(0);
            }
        }


    }

    @Override
    public void loop() {

    }
}
