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

import static org.firstinspires.ftc.teamcode.ballbot.EnhancedGamepad.Button.*;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.ballbot.BallBot;
import org.firstinspires.ftc.teamcode.ballbot.EnhancedGamepad;
import org.firstinspires.ftc.teamcode.ballbot.Launcher;

@TeleOp(name="ITD BallBot", group="Ballbot")
//@Disabled
public class TeleBallBot extends LinearOpMode {
    private BallBot bot = null;
    private EnhancedGamepad epad2 = null;
    private EnhancedGamepad epad1 = null;
    @Override
    public void runOpMode() {
        Launcher launcher = new Launcher(hardwareMap, this);
        launcher.initHardware();
        telemetry.addData("Status", "Initialized");
        telemetry.update();
        launcher.resetPositions();
        // create a new robot object.
        bot = new BallBot(hardwareMap, this);

        // get an enhanced gamepad for driver #2.
        epad2 = new EnhancedGamepad(gamepad2);
        // get an enhanced gamepad for driver #1.
        epad1 = new EnhancedGamepad(gamepad1);
        bot.resetAngles();
        // wait for start command from driver hub.
        waitForStart();

        // loop until opmode is stopped.
        while (opModeIsActive()) {
            // get gamepad input for mecanum drive.
            double drive = -gamepad1.left_stick_y;
            double strafe = gamepad1.left_stick_x;
            double twist = gamepad1.right_stick_x;

            // scale power?
            double scale = Math.abs(1 - gamepad1.right_trigger);
            drive = scale * drive;
            strafe = scale * strafe;
            twist = scale * twist;

            // update the mecanum drive.
            bot.drive(drive, strafe, twist);

            // use enhanced gamepad to detect if buttons were just pressed.
            // update the enhanced gamepad data.
            epad2.poll();
            epad1.poll();
            if (epad1.justPressed(DPAD_DOWN)) {
                launcher.tiltDown();
            }
            if (epad1.justPressed(DPAD_UP)) {
                launcher.tiltUp();
            }
            //Prevents balls being released when the spinner is not moving
            if (epad1.justPressed(A)) {
                if (launcher.getSpinnerSpeed() > 0 && launcher.getSpinnerState()) {
                launcher.releaseBall();
                }
            }
            if (epad1.justPressed(RIGHT_BUMPER)) {
                launcher.speedUp();
                launcher.updateSpinner();
            }
            if (epad1.justPressed(LEFT_BUMPER)) {
                launcher.speedDown();
                launcher.updateSpinner();
            }
            if (epad1.pressed(START) && epad1.pressed(BACK)) {
                launcher.toggleSpinner();
                launcher.updateSpinner();
            }
            if (epad1.justPressed(RIGHT_STICK)) {
                launcher.turnSpinnerOff();
                launcher.updateSpinner();
            }
            if (epad1.justPressed(Y)) {
                launcher.toggleIntake();
            }
            if (epad1.justPressed(X)) {
                launcher.toggleLift();
            }
            double startTime = 0;
            int timestate = 0;
            if (launcher.calculateBalls(bot.getDistance()) == 5) {
                if (timestate == 0) {
                    timestate = 1;
                    startTime = this.getRuntime();
                } else {
                    double currTime = this.getRuntime();
                    if (currTime - startTime > 2000 && launcher.calculateBalls(bot.getDistance()) == 5) {
                        launcher.liftOff();
                    }
                }

            }


            bot.updateAngles();
            telemetry.addData("Tilt position ", launcher.currentTilt);
            telemetry.addData("integrated angle", bot.integratedAngle);
            telemetry.addData("spinner target speed", launcher.spinnerTargetSpeed);
            telemetry.addData("distance", bot.getDistance());
            telemetry.addData("Balls: ", launcher.calculateBalls(bot.getDistance()));
            telemetry.update();
        }
    }
}
