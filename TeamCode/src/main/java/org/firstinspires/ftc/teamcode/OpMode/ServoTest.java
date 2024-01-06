package org.firstinspires.ftc.teamcode.OpMode;

import androidx.core.math.MathUtils;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.Util.MathUtil;

import java.io.BufferedReader;
import java.io.File;
import java.io.FileNotFoundException;
import java.io.FileReader;
import java.io.IOException;
import java.io.InputStream;
import java.util.ArrayList;
import java.util.Scanner;

@TeleOp(name = "Servo on Left Control Stick", group = "Tests")
public class ServoTest extends LinearOpMode {

    double servoMin;
    double servoMax;

    @Override
    public void runOpMode() {

        String filePath = "/sdcard/servos";
        reloadSettings(filePath);

        ArrayList<Servo> servos = new ArrayList<>();
        hardwareMap.servo.iterator().forEachRemaining(servos::add);

        waitForStart();
        double servoTarget;

        while (opModeIsActive()) {

            servoTarget = MathUtil.remap(-1, 1, servoMin, servoMax, gamepad1.left_stick_x);
            for (Servo servo : servos) {
                servo.setPosition(servoTarget);
                telemetry.addData("Servo " + servo.getPortNumber(), servo.getPosition());
            }

            if (gamepad1.a) {
                reloadSettings(filePath);
            }

            telemetry.update();

        }
    }

    public void reloadSettings(String path) {
        Scanner scanner;
        try {
            scanner = new Scanner(new FileReader(path));
        } catch (FileNotFoundException e) {
            throw new RuntimeException(e);
        }

        servoMin = scanner.nextDouble();
        servoMax = scanner.nextDouble();
    }
}
