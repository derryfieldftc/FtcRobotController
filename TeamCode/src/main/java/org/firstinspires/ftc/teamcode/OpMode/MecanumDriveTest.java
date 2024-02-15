package org.firstinspires.ftc.teamcode.OpMode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

import org.firstinspires.ftc.teamcode.Helper.MecanumDrive;

@TeleOp(name="MecanumDriveTest", group="Tests")
public class MecanumDriveTest extends LinearOpMode {

    public static final String RIGHT_FRONT_MOTOR_NAME = "motorFR";
    public static final String LEFT_FRONT_MOTOR_NAME = "motorFL";
    public static final String RIGHT_REAR_MOTOR_NAME = "motorBR";
    public static final String LEFT_REAR_MOTOR_NAME = "motorBL";
    public static final String LINEAR_SLIDE_MOTOR_NAME = "slide";
    public static final String INTAKE_MOTOR_NAME = "intake";
    public static final String IMU_NAME = "imu";
    public static final double ENCODER_RESOLUTION = 3895.9;
    public static final double WHEEL_DIAMETER_CM = 9.6;
    public static final int MINIMUM_SLIDE_POSITION = -70;
    public static final int MAXIMUM_SLIDE_POSITION = 4141;

    @Override
    public void runOpMode() {

//        MecanumDrive mecanum = new MecanumDrive(
//            hardwareMap,
//            RIGHT_FRONT_MOTOR_NAME,
//            LEFT_FRONT_MOTOR_NAME,
//            RIGHT_REAR_MOTOR_NAME,
//            LEFT_REAR_MOTOR_NAME,
//                IMU_NAME,
//                ENCODER_RESOLUTION,
//                WHEEL_DIAMETER_CM,
//                this
//        );

        waitForStart();

        double forward, strafe, rotate, scale; // Drive variables

        // Set motors
        // DcMotor slideMotor = (DcMotor)hardwareMap.get(LINEAR_SLIDE_MOTOR_NAME);
        DcMotor intakeMotor = (DcMotor)hardwareMap.get(INTAKE_MOTOR_NAME);

        //Set motor behaviors
//        intakeMotor.setDirection(DcMotorSimple.Direction.REVERSE);
//        slideMotor.setDirection(DcMotorSimple.Direction.REVERSE);
//        slideMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
//        slideMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        double slidePower = 0.5;
        double intakePower = 1;
        int slidePosition = 0;
        int slideSpeed = 10;

        while (opModeIsActive()) {

            forward = -gamepad1.left_stick_y; // Up is negative; we want up to be positive, so we *(-1)
            strafe = gamepad1.left_stick_x; // Perfect child, no flip
            rotate = -gamepad1.right_stick_x; // Positive is CCW; we want positive to be CW, so we *(-1)
            scale = 0.8;

            // mecanum.drive(forward, strafe, rotate, scale);

            // Intake Logic
            intakeMotor.setPower(intakePower * (gamepad2.right_trigger - gamepad2.left_trigger));

            // Slide Logic
            if (gamepad2.dpad_up) {
                slidePosition = MAXIMUM_SLIDE_POSITION;
                slidePower = 0.5;
            }
            else if (gamepad2.dpad_down) {
                slidePosition = MINIMUM_SLIDE_POSITION;
                slidePower = -0.5;
            }
            else slidePower = 0;
//            slideMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//            slideMotor.setTargetPosition(slidePosition);
//            slideMotor.setPower(slidePower);

//            telemetry.addData("forward", forward);
//            telemetry.addData("strafe", strafe);
//            telemetry.addData("rotate", rotate);
//            telemetry.addData("slide position", slideMotor.getCurrentPosition());
            telemetry.update();
        }
    }

//    /**
//    Blocking function that uses encoders to spin a motor to a desired encoder position
//     **/
//    public void runToPosition(DcMotor motor, int target, int power) {
//        motor.setTargetPosition(target);
//        while (motor.getCurrentPosition() <= target) {
//            motor.setPower(power);
//        }
//        while (motor.getCurrentPosition() >= target) {
//            motor.setPower(-power);
//        }
//    }


}
