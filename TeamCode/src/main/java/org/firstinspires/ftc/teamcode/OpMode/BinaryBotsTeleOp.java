package org.firstinspires.ftc.teamcode.OpMode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.TouchSensor;

import org.firstinspires.ftc.teamcode.Helper.MecanumDrive;
import org.firstinspires.ftc.teamcode.Helper.ServoMechanism;

@TeleOp(name="Binary Bots TeleOp")
public class BinaryBotsTeleOp extends LinearOpMode {

    public static final String RIGHT_FRONT_MOTOR_NAME = "motorFR";
    public static final String LEFT_FRONT_MOTOR_NAME = "motorFL";
    public static final String RIGHT_REAR_MOTOR_NAME = "motorBR";
    public static final String LEFT_REAR_MOTOR_NAME = "motorBL";
    public static final String INTAKE_MOTOR_NAME = "intake";
    public static final String LINEAR_SLIDE_MOTOR_NAME = "slide";
    public static final String LIMIT_SWITCH_NAME = "limit";
    public static final String IMU_NAME = "imu";
    public static final double ENCODER_RESOLUTION = 1120;
    public static final double WHEEL_DIAMETER_CM = 7.4;
    public static final double MAX_SLIDE_POSITION = 2371;

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

        DcMotor intakeMotor = hardwareMap.dcMotor.get(INTAKE_MOTOR_NAME);
        intakeMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        DcMotor slideMotor = hardwareMap.dcMotor.get(LINEAR_SLIDE_MOTOR_NAME);
        slideMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        slideMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        slideMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        // float MINIMUM_SLIDE_POSITION = slideMotor.getCurrentPosition();

        TouchSensor limit = hardwareMap.touchSensor.get(LIMIT_SWITCH_NAME);

        ServoMechanism.Builder clawBuilder = new ServoMechanism.Builder()
                .setServo(hardwareMap, "claw")
                .addState("clasped", 0.0)
                .addState("released", 0.75);
        ServoMechanism clawServo = clawBuilder.build();
        clawServo.setStateByName("released");

        ServoMechanism.Builder rotatorBuilder = new ServoMechanism.Builder()
                .setServo(hardwareMap, "rotator")
                .addState("scoring", 0.1)
                .addState("collecting", 0.92);
        ServoMechanism rotatorServo = rotatorBuilder.build();
        rotatorServo.setStateByName("collecting");

        double forward, strafe, rotate, scale; // Drive variables
        double intakePower = 1.0;
        double slidePower = 1.0; // Slide variables
        double slideIncrement = 80.0;

        boolean currentClaw, previousClaw = false; // Claw variables
        boolean currentRotator, previousRotator = false; // Rotator variables

        waitForStart();

        while (opModeIsActive()) {
            /*
            Controls:
                GamePad1:
                    LeftStick   = Translational Motion
                    RightStickX = Rotational Motion
                GamePad2:
                    Triggers    = Control Intake
                    A           = Toggle Claw
                    Y           = Toggle Rotator Servo
                    DPadUp      = Move Slide Up
                    DPadDown    = Move Slide Down
             */

            // Drive Logic
            forward = -gamepad1.left_stick_y; // Up is negative; we want up to be positive, so we *(-1)
            strafe = gamepad1.left_stick_x; // Perfect child, no flip
            rotate = -gamepad1.right_stick_x; // Positive is CCW; we want positive to be CW, so we *(-1)
            scale = 0.8;
            mecanum.drive(forward, strafe, rotate, scale);

            // Intake Logic
            intakeMotor.setPower(intakePower * (gamepad2.right_trigger - gamepad2.left_trigger));

            // Slide Logic
            double slideTarget = slideMotor.getCurrentPosition();
            if (gamepad2.dpad_up && slideMotor.getCurrentPosition() < MAX_SLIDE_POSITION) {
                // slideTarget = slideMotor.getCurrentPosition() + 1; // 1 should be slideSpeed (or something like that)!
                slideTarget = slideMotor.getCurrentPosition() + slideIncrement;
            }
            else if (gamepad2.dpad_down && !limit.isPressed()) {
                // slideTarget = slideMotor.getCurrentPosition() - 1;
                slideTarget = slideMotor.getCurrentPosition() - slideIncrement;
            }
//            if (gamepad2.dpad_up) { slideSpeed = slidePower; }
//            else if (gamepad2.dpad_down) { slideSpeed = -slidePower; }
            slideMotor.setTargetPosition((int)slideTarget);
            slideMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            slideMotor.setPower(slidePower);

            // Claw State Machine
            currentClaw = gamepad2.a;
            if (currentClaw && !previousClaw) {
                previousClaw = currentClaw;
                if (clawServo.getCurrentState().stateName.equals("clasped")) { clawServo.setStateByName("released"); }
                else if (clawServo.getCurrentState().stateName.equals("released")) { clawServo.setStateByName("clasped"); }
            }
            else if (!currentClaw && previousClaw) {
                previousClaw = currentClaw;
            }

            // Rotator State Machine
            currentRotator = gamepad2.y;
            if (currentRotator && !previousRotator) {
                previousRotator = currentRotator;
                if (rotatorServo.getCurrentState().stateName.equals("collecting")) { rotatorServo.setStateByName("scoring"); }
                else if (rotatorServo.getCurrentState().stateName.equals("scoring")) { rotatorServo.setStateByName("collecting"); }
            }
            else if (!currentRotator && previousRotator) {
                previousRotator = currentRotator;
            }

            telemetry.addData("forward", forward);
            telemetry.addData("strafe", strafe);
            telemetry.addData("rotate", rotate);
            telemetry.addData("slide position", slideMotor.getCurrentPosition());
            telemetry.addData("clawState", clawServo.getCurrentState().stateName);
            telemetry.addData("rotatorState", rotatorServo.getCurrentState().stateName);
            telemetry.update();
        }

    }

}
