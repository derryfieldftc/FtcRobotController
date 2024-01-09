package org.firstinspires.ftc.teamcode.OpMode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.Helper.MecanumDrive;

@Autonomous(name="Encoder Test", group="Tests")
public class EncoderTest extends LinearOpMode {

    public static final String RIGHT_FRONT_MOTOR_NAME = "motorFR";
    public static final String LEFT_FRONT_MOTOR_NAME = "motorFL";
    public static final String RIGHT_REAR_MOTOR_NAME = "motorBR";
    public static final String LEFT_REAR_MOTOR_NAME = "motorBL";
    public static final String IMU_NAME = "imu";
    /**
     * 1 full revolution of the wheel
     */
    public static final double ENCODER_RESOLUTION = 1120;
    public static final double WHEEL_DIAMETER_CM = 8;

    @Override
    public void runOpMode() {

        MecanumDrive mecanum = new MecanumDrive(
                hardwareMap,
                RIGHT_FRONT_MOTOR_NAME,
                LEFT_FRONT_MOTOR_NAME,
                RIGHT_REAR_MOTOR_NAME,
                LEFT_REAR_MOTOR_NAME,
                IMU_NAME,
                ENCODER_RESOLUTION,
                WHEEL_DIAMETER_CM,
                this
        );

        waitForStart();

//        mecanum.turnUsingIMU(45, 0.5);
//        sleep(500);
//        mecanum.turnUsingIMU(-45, 0.5);
//        sleep(500);
//        mecanum.turnUsingIMU(90, 0.5);
//        sleep(500);
//        mecanum.turnUsingIMU(-90, 0.5);
//        sleep(500);
//        mecanum.turnUsingIMU(270, 0.5);
//        sleep(500);
//        mecanum.turnUsingIMU(-270, 0.5);
//        sleep(500);
//        mecanum.turnUsingIMU(360, 0.5);
//        sleep(500);
//        mecanum.turnUsingIMU(-360, 0.5);
//        sleep(500);
        mecanum.turnUsingIMU(450, 1);
        sleep(500);
        mecanum.turnUsingIMU(-450, 1);
        sleep(500);
        mecanum.turnUsingIMU(720, 1);
        sleep(500);
        mecanum.turnUsingIMU(-720, 1);
        sleep(500);
        mecanum.turnUsingIMU(1080, 1);
        sleep(500);
        mecanum.turnUsingIMU(-1080, 1);

//        mecanum.driveCentimetersStrafe(122, 0.3);
//        sleep(1000);
//        mecanum.driveCentimetersStrafe(-122, 0.3);

        // If program ends immediately, motor ZeroPowerBrake doesn't work
        sleep(500);

    }

}
