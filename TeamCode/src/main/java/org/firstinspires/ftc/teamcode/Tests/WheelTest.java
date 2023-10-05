package org.firstinspires.ftc.teamcode.Tests;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.util.ElapsedTime;

@TeleOp(name="WheelTest")
public class WheelTest extends LinearOpMode {

        @Override
        public void runOpMode() throws InterruptedException {

            DcMotorEx lfMotor = hardwareMap.get(DcMotorEx.class, "LFMotor");
            DcMotorEx rfMotor = hardwareMap.get(DcMotorEx.class, "RFMotor");
            DcMotorEx lrMotor = hardwareMap.get(DcMotorEx.class, "LRMotor");
            DcMotorEx rrMotor = hardwareMap.get(DcMotorEx.class, "RRMotor");

            rfMotor.setDirection(DcMotorSimple.Direction.REVERSE);
            rrMotor.setDirection(DcMotorSimple.Direction.REVERSE);

            waitForStart();

            while (opModeIsActive()) {
                testAWheel(lfMotor, "LeftFront");
                testAWheel(lrMotor, "LeftRear");
                testAWheel(rfMotor, "RightFront");
                testAWheel(rrMotor, "RightRear");
            }
        }

        public void testAWheel (DcMotorEx motor, String motorName) {
            if (opModeIsActive()) {
                motor.setPower(0.5);
                telemetryDashboard(motor, motorName);
                motor.setPower(-0.5);
                telemetryDashboard(motor, motorName);
                motor.setPower(0.0);
            }
        }

        public void telemetryDashboard(DcMotorEx motor, String motorName) {
            ElapsedTime timer = new ElapsedTime();
            while (timer.milliseconds() < 2000 && opModeIsActive()) {
                telemetry.addData("Motor", motorName);
                telemetry.addData("Power", motor.getPower());
                telemetry.addData("Encoder", motor.getVelocity());
                telemetry.update();
                idle();
            }
        }
    }
