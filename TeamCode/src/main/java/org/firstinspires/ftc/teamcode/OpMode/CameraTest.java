package org.firstinspires.ftc.teamcode.OpMode;


import static org.firstinspires.ftc.teamcode.Helper.AssignMotors.*;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.checkerframework.checker.units.qual.C;
import org.firstinspires.ftc.teamcode.Helper.Camera;
import org.firstinspires.ftc.teamcode.Helper.MecanumDrive;
@Autonomous(name = "BlueCenterStage")
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

        ewok = new MecanumDrive(
                hardwareMap,
                RIGHT_FRONT_MOTOR_NAME,
                LEFT_FRONT_MOTOR_NAME,
                RIGHT_REAR_MOTOR_NAME,
                LEFT_REAR_MOTOR_NAME,
                INTAKE_MOTOR_NAME,
                IMU_NAME,
                ENCODER_RESOLUTION,
                WHEEL_DIAMETER_CM,
                this
        );

        Camera camera = new Camera(USE_WEBCAM, CAMERA_NAME, TFOD_MODEL_ASSET, LABELS, hardwareMap);

        telemetry.addLine("Waiting for Start...");
        telemetry.update();

        waitForStart();

        while (opModeIsActive()) {
            // String label = camera.recognizeByLabel();
            telemetry.addData("Labels: ", camera.recognizeByLabel());
            if (camera.recognizeByBoolean("Red Prop")) {
                telemetry.addLine("Found a RED Prop");
            }
            else if (camera.recognizeByBoolean("Blue Prop")) {
                telemetry.addLine("Found a BLUE Prop");
            }
            telemetry.addLine("Found No Prop");
            telemetry.update();
        }
//        mecanum.driveCentimetersForward(65, 1);
//        if (camera.detectPixel(10, 300, 2)){
//            mecanum.driveCentimetersForward(40, 1);
//            sleep(100);
//            mecanum.driveCentimetersForward(-35, 1);
//            mecanum.turnUsingIMU(81, 0.5);
//            mecanum.driveCentimetersForward(200, 1);
//        }
//        else {
//            mecanum.turnUsingIMU(-45, 0.5);
//            if (camera.detectPixel(10, 300, 2)){
//                mecanum.driveCentimetersForward(40, 1);
//                sleep(100);
//                mecanum.driveCentimetersForward(-40, 1);
//                mecanum.turnUsingIMU(126, 0.5);
//                mecanum.driveCentimetersForward(200, 1);
//            }
//            else {
//                mecanum.turnUsingIMU(45, 0.5);
//                mecanum.driveCentimetersStrafe(-40, 1);
//                mecanum.turnUsingIMU(81, 0.5);
//                mecanum.driveCentimetersForward(180, 1);
//            }
//        }


        //mecanum.driveCentimetersForward(40, 0.3);




        // If program ends immediately, motor ZeroPowerBrake doesnt work
        sleep(500);

    }

}