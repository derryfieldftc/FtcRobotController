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
import org.firstinspires.ftc.teamcode.binarybot.Manipulator;

@Autonomous(name="Test auto High Goal", group="BinaryBot")
//@Disabled
public class TestScoreHighGoal extends LinearOpMode {

    // Declare OpMode members.
    private BinaryBot bot = null;
    @Override
    public void runOpMode() {
        telemetry.addData("Status", "Initialized");
        telemetry.update();

        // create a new robot object.
        bot = new BinaryBot(hardwareMap, this);

        //    calibrate encoders for robot
        bot.calibrateOdometry();

        // reset positions on manipulator.
        bot.manipulator.resetPositions();

        // get a reference to the manipulator.
        Manipulator manipulator = bot.manipulator;
        //calibrate encoders for manipulator
        manipulator.calibrate();
        // wait for start command from driver hub.
        waitForStart();

        // move off the wall by strafing to the left.
        if (opModeIsActive()) {
//            moves 6 inches to the left at power .4
            bot.measuredStrafe(0.3, -6);

            // loop until done.
            while(opModeIsActive() &&  bot.update()) {
                // send telemetry.
                telemetry.addData("state", bot.state);
                telemetry.addData("tgt pos", bot.targetPos);
                telemetry.addData("curr pos", bot.currentPos);
                telemetry.update();
            }
        }

        while (opModeIsActive()){
            telemetry.addData("Status", "Waiting for stop.......");
        }
    }
}
