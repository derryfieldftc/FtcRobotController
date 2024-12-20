/* Copyright (c) 2017 FIRST. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted (subject to the limitations in the disclaimer below) provided that
 * the following conditions are met:
 *
 * Redistributions of source code must retain the above copyright notice, this list
 * of conditions and the following disclaimer.
 *
 * Redistributions in binary form must reproduce the above copyright notice, this
 * list of conditions and the following disclaimer in the documentation and/or
 * other materials provided with the distribution.
 *
 * Neither the name of FIRST nor the names of its contributors may be used to endorse or
 * promote products derived from this software without specific prior written permission.
 *
 * NO EXPRESS OR IMPLIED LICENSES TO ANY PARTY'S PATENT RIGHTS ARE GRANTED BY THIS
 * LICENSE. THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
 * THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

package org.firstinspires.ftc.teamcode.teleop;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.binarybot.BinaryBot;
import org.firstinspires.ftc.teamcode.binarybot.EnhancedGamepad;
import static org.firstinspires.ftc.teamcode.binarybot.EnhancedGamepad.Button.*;

@TeleOp(name="TeleOp Binary Bot", group="BinaryBot")
//@Disabled
public class TeleBinaryBot extends LinearOpMode {

    // Declare OpMode members.
    private BinaryBot bot = null;
    private EnhancedGamepad epad2 = null;

    @Override
    public void runOpMode() {
        telemetry.addData("Status", "Initialized");
        telemetry.update();

        // create a new robot object.
        bot = new BinaryBot(hardwareMap, this);

        // get an enhanced gamepad for driver #2.
        epad2 = new EnhancedGamepad(gamepad2);

        // wait for start command from driver hub.
        waitForStart();

        // loop until opmode is stopped.
        while (opModeIsActive()) {
            // get gamepad input for mecanum drive.
            double drive = -gamepad1.left_stick_y;
            double strafe = gamepad1.left_stick_x;
            double twist = gamepad1.right_stick_x;

            // update the mecanum drive.
            bot.drive(drive, strafe, twist);

            // use enhanced gamepad to detect if buttons were just pressed.
            // update the enhanced gamepad data.
            epad2.poll();

            // extend or retract the slide?
            if (epad2.justPressed(DPAD_UP)) {
                bot.manipulator.extendSlide();
            }

            if (epad2.justPressed(DPAD_DOWN)) {
                bot.manipulator.retractSlide();
            }

            // trim the slide using the left joystick (y direction).
            bot.manipulator.trimSlide(-epad2.gamepad.left_stick_y);

            // figure out tilt of bucket.
            bot.manipulator.tiltBucket(epad2.gamepad.right_trigger);

            // trim the shoulder.
            bot.manipulator.trimShoulder(-epad2.gamepad.right_stick_y);

            // toggle the wrist?
            if (epad2.justPressed(RIGHT_BUMPER)) {
                bot.manipulator.toggleWrist();
            }

            // toggle the claw?
            if (epad2.justPressed(LEFT_BUMPER)) {
                bot.manipulator.toggleClaw();
            }

            // transfer sample?
            if (epad2.justPressed(B)) {
                bot.manipulator.transfer();
            }

            // tilt left or right?
            if (epad2.justPressed(DPAD_LEFT)) {
                bot.manipulator.tiltLeft();
            }

            if (epad2.justPressed(DPAD_RIGHT)) {
                bot.manipulator.tiltRight();
            }

            if (epad2.justPressed(DPAD_LEFT)) {
                //bot.manipulator.tilt.setPosition(0.75);
                bot.manipulator.elbow.setPosition(0);
            }

            if (epad2.justPressed(DPAD_RIGHT)) {
                //bot.manipulator.tilt.setPosition(0);
                bot.manipulator.elbow.setPosition(1.0);
            }

            if (epad2.justPressed(LEFT_STICK)) {
                bot.manipulator.tilt.setPosition(0.75);
                //bot.manipulator.elbow.setPosition(0);
            }

            if (epad2.justPressed(RIGHT_STICK)) {
                bot.manipulator.tilt.setPosition(0);
                //bot.manipulator.elbow.setPosition(1.0);
            }

            // trim the shoulder using the right joystick (y direction).
            //bot.manipulator.trimShoulder(-epad2.gamepad.right_stick_y);

            // turn on shoulder motor?
            // should we make this that they have to push both joystick buttons
            // (to avoid accidental shoulder movement)?
//            if (epad2.justPressed(LEFT_STICK)) {
//                bot.manipulator.activateShoulder();
//            }

            telemetry.addData("shoulder curr pos", bot.manipulator.shoulder.getCurrentPosition());
            telemetry.addData("shoulder tgt pos", bot.manipulator.shoulder.getTargetPosition());
            telemetry.addData("slide curr pos", bot.manipulator.slide.getCurrentPosition());
            telemetry.addData("slide tgt pos", bot.manipulator.slide.getTargetPosition());
            telemetry.update();
        }
    }
}
