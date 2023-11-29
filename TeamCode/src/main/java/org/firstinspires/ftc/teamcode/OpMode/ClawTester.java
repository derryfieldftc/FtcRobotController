package org.firstinspires.ftc.teamcode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.Helper.ClawMechanism;

@TeleOp(name = "ClawTest", group = "Tests")
public class ClawTester extends LinearOpMode {

    public void runOpMode() {

        telemetry.addData("Status", "Initialized");

        ClawMechanism claw = new ClawMechanism(
            hardwareMap.servo.get("Servo2"),
            hardwareMap.servo.get("Servo0")
        );

        telemetry.update();
        waitForStart();

        while (opModeIsActive()) {

            if (gamepad1.a) claw.clamp();
            else if (gamepad1.b) claw.release();
            else this.sleep(100);

            telemetry.addData("Claw State", claw.getCurrentState());

            telemetry.update();

        }

    }
}
