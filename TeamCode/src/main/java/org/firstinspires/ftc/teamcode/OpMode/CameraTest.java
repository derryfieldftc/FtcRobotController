package org.firstinspires.ftc.teamcode.OpMode;


import static org.firstinspires.ftc.teamcode.Helper.AssignMotors.*;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.Helper.Camera;
import org.firstinspires.ftc.teamcode.Helper.MecanumDrive;
@Autonomous(name = "BlueCenterStage")
public class CameraTest extends LinearOpMode {
    public static final String CAMERA_NAME = "Webcam 1";
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

        Camera camera = new Camera(hardwareMap, CAMERA_NAME, this);

        waitForStart();

        ewok.driveCentimetersForward(65, 1);
        if (camera.detectPixel(10, 300, 2)){
            ewok.driveCentimetersForward(40, 1);
            sleep(100);
            ewok.driveCentimetersForward(-35, 1);
            ewok.turnUsingIMU(81, 0.5);
            ewok.driveCentimetersForward(200, 1);
        }
        else {
            ewok.turnUsingIMU(-45, 0.5);
            if (camera.detectPixel(10, 300, 2)){
                ewok.driveCentimetersForward(40, 1);
                sleep(100);
                ewok.driveCentimetersForward(-40, 1);
                ewok.turnUsingIMU(126, 0.5);
                ewok.driveCentimetersForward(200, 1);
            }
            else {
                ewok.turnUsingIMU(45, 0.5);
                ewok.driveCentimetersStrafe(-40, 1);
                ewok.turnUsingIMU(81, 0.5);
                ewok.driveCentimetersForward(180, 1);
            }
        }


        //mecanum.driveCentimetersForward(40, 0.3);




        // If program ends immediately, motor ZeroPowerBrake doesnt work
        sleep(500);

    }

}