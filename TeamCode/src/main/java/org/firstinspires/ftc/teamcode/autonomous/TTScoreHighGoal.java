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

@Autonomous(name="TT High Goal", group="BinaryBot")
//@Disabled
public class TTScoreHighGoal extends LinearOpMode {

    // Declare OpMode members.
    private BinaryBot bot = null;
    @Override
    public void runOpMode() {
        telemetry.addData("Status", "Initialized");
        telemetry.update();

        // create a new robot object.
        bot = new BinaryBot(hardwareMap, this);

        //    calibrate encoders for robot
        bot.calibrate();

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
//            moves 6 inches to the left at power .3
            bot.measuredStrafe(Manipulator.MOTOR_SPEED, -6);

            // loop until done.
            while(opModeIsActive() &&  bot.measuredUpdate()) {
                // send telemetry.
                telemetry.addData("state", bot.measuredState);
                telemetry.addData("tgt pos", bot.tgtPos);
                telemetry.addData("curr pos", bot.currPos);
                telemetry.update();
            }
        }

        // go backwards to approach goal.
        if (opModeIsActive()) {
            bot.measuredDrive(.3, -55);

            // loop until done.
            while(opModeIsActive() &&  bot.measuredUpdate()) {
                // send telemetry.
                telemetry.addData("state", bot.measuredState);
                telemetry.addData("tgt pos", bot.tgtPos);
                telemetry.addData("curr pos", bot.currPos);
                telemetry.update();
            }
        }

        // turn CCW.
        if (opModeIsActive()) {
            bot.measuredTurn(Manipulator.MOTOR_SPEED, -40);

            // loop until done.
            while(opModeIsActive() &&  bot.measuredUpdate()) {
                // send telemetry.
                telemetry.addData("state", bot.measuredState);
                telemetry.addData("tgt angle", bot.tgtAngle);
                telemetry.addData("curr angle", bot.integratedAngle);
                telemetry.update();
            }
        }
        // moves 2 inches back to buckets
        if (opModeIsActive()) {
            bot.measuredDrive(Manipulator.MOTOR_SPEED, -2);
            // loop until done.
            while(opModeIsActive() &&  bot.measuredUpdate()) {
                // send telemetry.
                telemetry.addData("state", bot.measuredState);
                telemetry.addData("tgt pos", bot.tgtPos);
                telemetry.addData("curr pos", bot.currPos);
                telemetry.update();
            }
        }
        // raise slide and dump.
        if (opModeIsActive()) {
            manipulator.startHighDump();

            // loop until high dump is done.
            while(opModeIsActive() &&  manipulator.update()) {
                // send telemetry.
                telemetry.addData("Status", "Doing high goal dump...");
                telemetry.update();
            }
        }
//        // lower slide half way
//        if (opModeIsActive()){
//            manipulator.slide.setTargetPosition(manipulator.SLIDE_MID_POSITION);
//
//            while(opModeIsActive() &&  manipulator.slide.isBusy()) {
//                // send telemetry.
//                telemetry.addData("Status", "Moving to slide to mid position...");
//                telemetry.update();
//            }
//        }
        //deploys shoulder
        if (opModeIsActive()){
            manipulator.deploy();

            while(opModeIsActive() &&  manipulator.update()) {
                //send telemetry
                telemetry.addData("Status", "Deploying manipulator...");
                telemetry.update();
            }
//             moves elbow/////////////////////////////////////////
            manipulator.updateElbow();
        }
//        retract slide
        if (opModeIsActive()){
            manipulator.tilt.setPosition(manipulator.TILT_DEPLOYED);
            manipulator.slide.setTargetPosition(manipulator.SLIDE_RETRACTED_POSITION);
            while(opModeIsActive()&& manipulator.slide.isBusy()){
                telemetry.addData("Status","Moving to slide bottom position...");
                telemetry.update();
            }
        }
//        if(opModeIsActive()){
//
//            // moves elbow
//            manipulator.updateElbow();
//            manipulator.tilt.setPosition(manipulator.TILT_DEPLOYED);
//            while(opModeIsActive() && manipulator.update()){
//                telemetry.addData("Status","Deploying stuff...");
//                telemetry.update();
//            }
//        }
        if(opModeIsActive()){
            bot.measuredTurn(Manipulator.MOTOR_SPEED,-55);
            while(opModeIsActive() && bot.measuredUpdate()){
                // send telemetry.
                telemetry.addData("state", bot.measuredState);
                telemetry.addData("tgt angle", bot.tgtAngle);
                telemetry.addData("curr angle", bot.integratedAngle);
                telemetry.update();
            }
        }

        if (opModeIsActive()) {
            bot.measuredDrive(Manipulator.MOTOR_SPEED, 1.5);
            // loop until done.
            while(opModeIsActive() &&  bot.measuredUpdate()) {
                // send telemetry.
                telemetry.addData("state", bot.measuredState);
                telemetry.addData("tgt pos", bot.tgtPos);
                telemetry.addData("curr pos", bot.currPos);
                telemetry.update();
            }
        }
        if (opModeIsActive()) {
            bot.measuredStrafe(Manipulator.MOTOR_SPEED, 7.5);
            // loop until done.
            while(opModeIsActive() &&  bot.measuredUpdate()) {
                // send telemetry.
                telemetry.addData("state", bot.measuredState);
                telemetry.addData("tgt pos", bot.tgtPos);
                telemetry.addData("curr pos", bot.currPos);
                telemetry.update();
            }
        }
        if(opModeIsActive()){
            manipulator.unrotateWrist();

            while(opModeIsActive() && bot.measuredUpdate()){
                telemetry.addData("Status","idk...");
                telemetry.update();
            }
        }

        // pick from the floor.
        if (opModeIsActive()) {
            manipulator.pickFromFloor();
            while (opModeIsActive() && manipulator.update()) {
                telemetry.addData("Status", "picking from floor");
                telemetry.update();
            }
        }

        if(opModeIsActive()){

            manipulator.transfer();
            while (opModeIsActive() && manipulator.update()) {
                telemetry.addData("Status", "transferring to bucket");
                telemetry.update();
            }
        }
        if(opModeIsActive()){
            manipulator.elbow.setPosition(Manipulator.ELBOW_DEPLOYED);
            manipulator.deploy();

            while(opModeIsActive() && manipulator.update()){
                telemetry.addData("Status","Deploying stuff...");
                telemetry.update();
            }
        }
        if (opModeIsActive()) {
            bot.measuredStrafe(Manipulator.MOTOR_SPEED, -7.5);
            // loop until done.
            while(opModeIsActive() &&  bot.measuredUpdate()) {
                // send telemetry.
                telemetry.addData("state", bot.measuredState);
                telemetry.addData("tgt pos", bot.tgtPos);
                telemetry.addData("curr pos", bot.currPos);
                telemetry.update();
            }
        }


        if (opModeIsActive()) {
            bot.measuredDrive(Manipulator.MOTOR_SPEED, -1.5);
            // loop until done.
            while(opModeIsActive() &&  bot.measuredUpdate()) {
                // send telemetry.
                telemetry.addData("state", bot.measuredState);
                telemetry.addData("tgt pos", bot.tgtPos);
                telemetry.addData("curr pos", bot.currPos);
                telemetry.update();
            }
        }
        if(opModeIsActive()){
            bot.measuredTurn(Manipulator.MOTOR_SPEED,55);
            while(opModeIsActive() && bot.measuredUpdate()){
                // send telemetry.
                telemetry.addData("state", bot.measuredState);
                telemetry.addData("tgt angle", bot.tgtAngle);
                telemetry.addData("curr angle", bot.integratedAngle);
                telemetry.update();
            }
        }
        if(opModeIsActive()){
        manipulator.startHighDump();
            while(opModeIsActive() && manipulator.update()){
                telemetry.addData("Status","Deploying stuff...");
                telemetry.update();
            }
        }


        while (opModeIsActive()){
            telemetry.addData("Status", "Waiting for stop.......");
        }
    }
}
