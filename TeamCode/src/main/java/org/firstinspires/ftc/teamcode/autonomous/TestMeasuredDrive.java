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

package org.firstinspires.ftc.teamcode.autonomous;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.binarybot.BinaryBot;
import org.firstinspires.ftc.teamcode.binarybot.EnhancedGamepad;
import org.firstinspires.ftc.teamcode.binarybot.Manipulator;

@Autonomous(name="ITD Test Measured Drive", group="BinaryBot")
//@Disabled
public class TestMeasuredDrive extends LinearOpMode {

    // Declare OpMode members.
    private BinaryBot bot = null;
    private EnhancedGamepad epad2 = null;

    @Override
    public void runOpMode() {
        telemetry.addData("Status", "Initialized");
        telemetry.update();

        // create a new robot object.
        bot = new BinaryBot(hardwareMap, this);

        // get a reference to the manipulator.
        Manipulator manipulator = bot.manipulator;

        // get an enhanced gamepad for driver #2.
        epad2 = new EnhancedGamepad(gamepad2);

        // wait for start command from driver hub.
        waitForStart();

        if (opModeIsActive()) {
            // drive forward 4 feet.
            bot.measuredDrive(0.3, 48);

            // loop until done traveling 2 feet.
            while(opModeIsActive() &&  bot.update()) {
                // send telemetry.
                telemetry.addData("state", bot.state);
                telemetry.addData("tgt pos", bot.tgtPos);
                telemetry.addData("curr pos", bot.currPos);
                telemetry.update();
            }
        }
    }
}
