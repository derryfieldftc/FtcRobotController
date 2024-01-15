package org.firstinspires.ftc.teamcode.OpMode;

import static org.firstinspires.ftc.teamcode.Helper.AssignMotors.*;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.Helper.MecanumDrive;

@TeleOp(name = "TeleOpMain")
public class TeleOpMain extends LinearOpMode {
    MecanumDrive d;

    //This function is executed when this Op Mode is selected from the Driver Station.
    @Override
    public void runOpMode() {
        // Put initialization blocks here:

        double forwardInput;
        double strafeInput;
        double rotateInput;
        double acceleratorA; // A magnitude of acceleration
        double deceleratorA; // A magnitude of deceleration
        double acceleratorB; // A magnitude of acceleration
        double deceleratorB; // A magnitude of deceleration
        boolean reverse;
        boolean intake;
        int direction = -1;


        d = new MecanumDrive(
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
        telemetry.update();
        waitForStart();
        if (opModeIsActive()) {
            // Put run blocks here.
            while (opModeIsActive()) {
                // Put loop blocks here.

                forwardInput = gamepad1.left_stick_y * direction; // Controls for moving back and forward.
                strafeInput = -gamepad1.left_stick_x * direction; // Controls for strafing.
                rotateInput = -gamepad1.right_stick_x; // Controls for pivoting.
                acceleratorA = gamepad2.right_trigger;
                deceleratorA = gamepad2.left_trigger;
                acceleratorB = gamepad2.right_trigger;
                deceleratorB = gamepad2.left_trigger;
                reverse = gamepad1.right_bumper;
                intake = gamepad2.a;

                // Drive code
                if ((acceleratorA != 0 && deceleratorA == 0)||(acceleratorB != 0 && deceleratorB == 0)) {  // Accelerate
                    d.drive(forwardInput, strafeInput, rotateInput, 1.2);
                    telemetry.addData("Acceleration","accelerating");
                }
                else if ((acceleratorA == 0 && deceleratorA != 0)||(acceleratorB == 0 && deceleratorB != 0)) {  // Decelerate
                    d.drive(forwardInput, strafeInput, rotateInput, 0.6);
                    telemetry.addData("Acceleration", "decelerating");
                }
                else {  // If both triggers are pressed, don't accelerate in either direction
                    d.drive(forwardInput, strafeInput, rotateInput,0.8);
                    telemetry.addData("Acceleration", "null");
                }
                if (reverse){
                    direction = 1;
                }
                else{
                    direction = -1;
                }
                if (intake){
                    d.intakeMotor.setPower(-1);
                }
                telemetry.update();
            }
        }
    }
}