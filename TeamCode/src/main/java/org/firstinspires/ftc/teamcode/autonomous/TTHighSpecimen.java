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
import com.qualcomm.robotcore.util.RobotLog;

import org.firstinspires.ftc.teamcode.binarybot.BinaryBot;
import org.firstinspires.ftc.teamcode.binarybot.EnhancedGamepad;
import org.firstinspires.ftc.teamcode.binarybot.Manipulator;

@Autonomous(name="TT High Specimen", group="BinaryBot")
//@Disabled
public class TTHighSpecimen extends LinearOpMode {

    // Declare OpMode members.
    private BinaryBot bot = null;
    private EnhancedGamepad epad2 = null;

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

        bot.manipulator.calibrate();
        // get a reference to the manipulator.
        Manipulator manipulator = bot.manipulator;

        // get an enhanced gamepad for driver #2.
        epad2 = new EnhancedGamepad(gamepad2);

        double distance = bot.getDistance();
        telemetry.addData("Distance", distance);
        telemetry.update();

        // wait for start command from driver hub.
        waitForStart();

        if (opModeIsActive()) {
            // distance to the submersible.
            bot.placeSpecimenHigh(distance-4);

            bot.update();
            while(opModeIsActive() && bot.update()) {
                RobotLog.aa("TIE", String.format("TIE: tgtPos = %d, curr pos = %d, dist to tgt = %.2f",
                        bot.targetPos,bot.currentPos, bot.getDistance()));

                // send telemetry.
                telemetry.addData("state", bot.state);
                telemetry.addData("slide", bot.manipulator.slide.getCurrentPosition());
                telemetry.addData("tgt pos", bot.targetPos);
                telemetry.addData("curr pos", bot.currentPos);
                telemetry.addData("Distance to tgt", bot.getDistance());
                telemetry.update();
            }
        }
        if(opModeIsActive()){
            //Turns robot around
            bot.measuredTurn(.4,180);
            while(opModeIsActive() && bot.update()){
                // send telemetry.
                telemetry.addData("state", bot.state);
                telemetry.addData("tgt angle", bot.tgtAngle);
                telemetry.addData("curr angle", bot.integratedAngle);
                telemetry.update();
            }
        }
        if (opModeIsActive()) {
            //Strafes to get past sub
            bot.measuredStrafe(.4, 20);
            // loop until done.
            while(opModeIsActive() &&  bot.update()) {
                // send telemetry.
                telemetry.addData("state", bot.state);
                telemetry.addData("tgt pos", bot.targetPos);
                telemetry.addData("curr pos", bot.currentPos);
                telemetry.update();
            }
        }
        if (opModeIsActive()) {
            //Drives to get past samples
            bot.measuredDrive(.4, 29);
            // loop until done.
            while(opModeIsActive() &&  bot.update()) {
                // send telemetry.
                telemetry.addData("state", bot.state);
                telemetry.addData("tgt pos", bot.targetPos);
                telemetry.addData("curr pos", bot.currentPos);
                telemetry.update();
            }
        }
        if (opModeIsActive()) {
            //Strafes to get samples
            bot.measuredStrafe(.4, 12);
            // loop until done.
            while(opModeIsActive() &&  bot.update()) {
                // send telemetry.
                telemetry.addData("state", bot.state);
                telemetry.addData("tgt pos", bot.targetPos);
                telemetry.addData("curr pos", bot.currentPos);
                telemetry.update();
            }
        }

        if (opModeIsActive()) {
            //Drives forward
            bot.measuredDrive(.4, -27);
            // loop until done.
            while(opModeIsActive() &&  bot.update()) {
                // send telemetry.
                telemetry.addData("state", bot.state);
                telemetry.addData("tgt pos", bot.targetPos);
                telemetry.addData("curr pos", bot.currentPos);
                telemetry.update();
            }
        }


    }
}
