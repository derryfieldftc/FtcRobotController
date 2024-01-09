package org.firstinspires.ftc.teamcode.OpMode;
import android.util.JsonReader;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.Helper.ClawMechanism;
import org.firstinspires.ftc.teamcode.Util.MathUtil;

import java.io.BufferedWriter;
import java.io.File;
import java.io.FileNotFoundException;
import java.io.FileReader;
import java.io.FileWriter;
import java.io.IOException;

@TeleOp(name = "ClawTest", group = "Tests")
public class ClawTester extends LinearOpMode {

    public void runOpMode() {

        ClawMechanism.Builder builder = new ClawMechanism.Builder();
        builder.setServo(hardwareMap, "Servo0");
        builder.addState("clamped", 0.3);
        builder.addState("released", 0.2);
        ClawMechanism claw = builder.build();


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

        }

    }
}