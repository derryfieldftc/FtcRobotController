package org.firstinspires.ftc.teamcode.OpMode;


import static org.firstinspires.ftc.teamcode.Helper.AssignMotors.*;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.checkerframework.checker.units.qual.C;
import org.firstinspires.ftc.robotcore.external.tfod.Recognition;
import org.firstinspires.ftc.teamcode.Helper.Camera;
import org.firstinspires.ftc.teamcode.Helper.MecanumDrive;

import java.util.Locale;

@TeleOp(name = "CameraTest", group = "Tests")
public class CameraTest extends LinearOpMode {

    public static final String RIGHT_FRONT_MOTOR_NAME = "motorFR";
    public static final String LEFT_FRONT_MOTOR_NAME = "motorFL";
    public static final String RIGHT_REAR_MOTOR_NAME = "motorBR";
    public static final String LEFT_REAR_MOTOR_NAME = "motorBL";
    public static final String IMU_NAME = "imu";
    private static final boolean USE_WEBCAM = true;
    public static final String CAMERA_NAME = "Webcam 1";
    private static final String TFOD_MODEL_ASSET = "TeamPropDetectionModel.tflite";
    // private static String TFOD_MODEL_FILE = "/sdcard/FIRST/tflitemodels/myCustomModel.tflite";
    private static final String[] LABELS = {
            "Blue Prop",
            "Red Prop"
    };

    /**
     * 1 full revolution of the wheel
     */

    public static final double ENCODER_RESOLUTION = 1120;
    public static final double WHEEL_DIAMETER_CM = 8;
    MecanumDrive ewok;
    @Override
    public void runOpMode() throws InterruptedException {

        Camera camera = new Camera(USE_WEBCAM, CAMERA_NAME, TFOD_MODEL_ASSET, LABELS, hardwareMap);

        telemetry.addLine("Waiting for Start...");
        telemetry.update();

        waitForStart();

        while (opModeIsActive()) {
            for (Recognition rec : camera.getRecognitions()) {
                double centerX = (rec.getLeft() + rec.getRight()) / 2.0;
                double centerY = (rec.getTop() + rec.getBottom()) / 2.0;
                telemetry.addLine(String.format(Locale.getDefault(), "%s at (%d, %d)", rec.getLabel(), centerX, centerY));
            }
        }

        sleep(500);

    }

}