package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

@TeleOp(name = "TeleOpBase")
public class TeleOpBase extends LinearOpMode {
    CenterStageRobot CenterStageRobot;

    //This function is executed when this Op Mode is selected from the Driver Station.
    @Override
    public void runOpMode() {
        // Put initialization blocks here:
        double forwardInput;
        double strafeInput;
        double rotateInput;
        double accelerator; // A magnitude of acceleration
        double decelerator; // A magnitude of deceleration

        int direction = 1;
        CenterStageRobot = new CenterStageRobot(this);
        CenterStageRobot.initHardware();
        waitForStart();
        if (opModeIsActive()) {
            //loop blocks here.
            while (opModeIsActive()) {
                forwardInput = gamepad1.left_stick_y * direction; // Controls for moving back and forward.
                strafeInput = gamepad1.left_stick_x * direction; // Controls for strafing.
                rotateInput = gamepad1.right_stick_x; // Controls for pivoting.
                accelerator = gamepad1.right_trigger;
                decelerator = gamepad1.left_trigger;

                //drive now
                if (accelerator != 0 && decelerator == 0) {  // Accelerate
                    CenterStageRobot.driveXYRB(strafeInput, forwardInput, rotateInput, accelerator, 1.0);
                } else if (accelerator == 0 && decelerator != 0) {  // Decelerate
                    CenterStageRobot.driveXYRB(strafeInput, forwardInput, rotateInput, decelerator, -1.0);
                } else {  // If both triggers are pressed, don't accelerate in either direction
                    CenterStageRobot.driveXYRB(strafeInput, forwardInput, rotateInput, 0.0, 1.0);
                }

            }
        }
    }
}