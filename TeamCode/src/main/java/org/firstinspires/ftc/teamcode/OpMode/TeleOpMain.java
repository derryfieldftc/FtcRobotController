package org.firstinspires.ftc.teamcode.OpMode;

import static org.firstinspires.ftc.teamcode.Helper.AssignMotors.*;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.Helper.ClawMechanism;
import org.firstinspires.ftc.teamcode.Helper.MecanumDrive;

@TeleOp(name = "TeleOpMain")
public class TeleOpMain extends LinearOpMode {
    MecanumDrive d;
    ClawMechanism c;

    //This function is executed when this Op Mode is selected from the Driver Station.
    @Override
    public void runOpMode() {
        // Put initialization blocks here:
        // Declaring the buttons that may quickly change:
        /*boolean liftUp;
        boolean liftDown;
        boolean liftOverride;
        double liftDecelerator;

        boolean intake;
        boolean outtake;

        boolean wristUp;
        boolean wristDown;
        boolean wristMiddle;
        double wristPosition;
        boolean setWristPosition;

        boolean lb;
        boolean rb;
        boolean y;
        boolean a; */

        double forwardInput;
        double strafeInput;
        double rotateInput;
        double accelerator; // A magnitude of acceleration
        double decelerator; // A magnitude of deceleration

      //  double liftPower;

        int direction = 1;

        // Declaring the former values of the buttons, so we can tell if they changed.

        d = new MecanumDrive(
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
        c = new ClawMechanism();
        telemetry.update();
        waitForStart();
        if (opModeIsActive()) {
            // Put run blocks here.
            while (opModeIsActive()) {
                // Put loop blocks here.
/*
                lb = gamepad1.left_bumper;
                rb = gamepad1.right_bumper;
                y = gamepad1.y;
                a = gamepad1.a;
*/
                forwardInput = gamepad1.left_stick_y * direction; // Controls for moving back and forward.
                strafeInput = gamepad1.left_stick_x * direction; // Controls for strafing.
                rotateInput = gamepad1.right_stick_x; // Controls for pivoting.
                accelerator = gamepad1.right_trigger;
                decelerator = gamepad1.left_trigger;
/*
                liftUp = gamepad2.right_bumper;
                liftDown = gamepad2.left_bumper;
                liftOverride = gamepad2.b;
                liftDecelerator = gamepad2.left_trigger;

                intake = gamepad2.a;
                outtake = gamepad2.y;

                wristUp = gamepad2.dpad_up;
                wristDown = gamepad2.dpad_down;
                wristMiddle = gamepad2.dpad_right;
                wristPosition = -gamepad2.left_stick_y;  // The joysticks are inverted to make up the positive direction and down the negative direction
                setWristPosition = gamepad2.x;
*/
                //liftPower = -gamepad2.right_stick_y;

                // Drive code
                if (accelerator != 0 && decelerator == 0) {  // Accelerate
                    d.drive(forwardInput, strafeInput, rotateInput, 1.5);
                    telemetry.addData("Acceleration","accelerating");
                }
                else if (accelerator == 0 && decelerator != 0) {  // Decelerate
                    d.drive(forwardInput, strafeInput, rotateInput, 0.6);
                    telemetry.addData("Acceleration", "decelerating");
                }
                else {  // If both triggers are pressed, don't accelerate in either direction
                    d.drive(forwardInput, strafeInput, rotateInput,1.0);
                    telemetry.addData("Acceleration", "null");
                }

                telemetry.update();
            }
        }
    }
}
