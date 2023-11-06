package org.firstinspires.ftc.teamcode.Tests;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorEx;

@TeleOp(name = "smErun")
public class SingleIntakeTest extends LinearOpMode {

    @Override
    public void runOpMode() throws InterruptedException {
        waitForStart();
        DcMotorEx spinMotor = hardwareMap.get(DcMotorEx.class, "SpinM");
        if (opModeIsActive()) {
            double trigger;
            //loop blocks here.
            while (opModeIsActive()) {
                trigger = gamepad1.right_trigger;
                spinMotor.setPower(-trigger);
            }
        }
    }
}
