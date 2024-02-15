package org.firstinspires.ftc.teamcode.OpMode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.Helper.MecanumDrive;

@TeleOp(name="Binary Bots TeleOp")
public class BinaryBotsTeleOp extends LinearOpMode {

    public static final String RIGHT_FRONT_MOTOR_NAME = "motorFR";
    public static final String LEFT_FRONT_MOTOR_NAME = "motorFL";
    public static final String RIGHT_REAR_MOTOR_NAME = "motorBR";
    public static final String LEFT_REAR_MOTOR_NAME = "motorBL";
    public static final String INTAKE_MOTOR_NAME = "intake";
    public static final String LINEAR_SLIDE_MOTOR_NAME = "slide";
    public static final String IMU_NAME = "imu";
    public static final double ENCODER_RESOLUTION = 1120;
    public static final double WHEEL_DIAMETER_CM = 7.4;

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

        DcMotor intakeMotor = (DcMotor)hardwareMap.get(INTAKE_MOTOR_NAME);
        DcMotor slideMotor = (DcMotor)hardwareMap.get(LINEAR_SLIDE_MOTOR_NAME);

        double forward, strafe, rotate, scale; // Drive variables
        double intakePower = 1;
        double slidePower = 1;

        while (opModeIsActive()) {

            forward = -gamepad1.left_stick_y; // Up is negative; we want up to be positive, so we *(-1)
            strafe = gamepad1.left_stick_x; // Perfect child, no flip
            rotate = -gamepad1.right_stick_x; // Positive is CCW; we want positive to be CW, so we *(-1)
            scale = 0.8;

            // Drive Logic
            mecanum.drive(forward, strafe, rotate, scale);

            // Slide Logic


            // Intake Logic
            if (gamepad1.dpad_down)
                intakeMotor.setPower(intakePower);
            else if (gamepad1.dpad_up)
                intakeMotor.setPower(-intakePower);
            else intakeMotor.setPower(0);

            telemetry.addData("forward", forward);
            telemetry.addData("strafe", strafe);
            telemetry.addData("rotate", rotate);
            telemetry.update();
        }

    }

}
