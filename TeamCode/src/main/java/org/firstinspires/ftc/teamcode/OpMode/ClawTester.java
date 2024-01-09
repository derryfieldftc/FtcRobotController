package org.firstinspires.ftc.teamcode.OpMode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.Helper.ServoMechanism;

@TeleOp(name = "ClawTest", group = "Tests")
public class ClawTester extends LinearOpMode {

    public void runOpMode() {

        ServoMechanism.Builder clawBuilder = new ServoMechanism.Builder();
        clawBuilder.setServo(hardwareMap, "Servo0");
        clawBuilder.addState("clamped", 0.22); // test and change servoPosition accordingly
        clawBuilder.addState("released", 0.3); // test and change servoPosition accordingly
        ServoMechanism claw = clawBuilder.build();

        ServoMechanism.Builder rotatorBuilder = new ServoMechanism.Builder();
        rotatorBuilder.setServo(hardwareMap, "Servo1");
        rotatorBuilder.addState("collecting", 0.0); // test and change servoPosition accordingly
        rotatorBuilder.addState("scoring", 1); // test and change servoPosition accordingly
        ServoMechanism rotator = rotatorBuilder.build();

        telemetry.update();
        waitForStart();

        while (opModeIsActive()) {

            telemetry.addData("Claw State", claw.getCurrentState().stateName);
            telemetry.update();

            if (gamepad1.a) {
                claw.setStateByName("clamped");
            } else if (gamepad1.b) {
                claw.setStateByName("released");
            }

            if (gamepad1.x) {
                rotator.setStateByName("collecting");
            } else if (gamepad1.y) {
                rotator.setStateByName("scoring");
            }

        }

    }
}