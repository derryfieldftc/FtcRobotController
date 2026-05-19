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
package org.firstinspires.ftc.teamcode.teach;


import android.graphics.Color;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.NormalizedColorSensor;
import com.qualcomm.robotcore.hardware.NormalizedRGBA;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.GamepadManager;

/*
 * This file contains an example of an iterative (Non-Linear) "OpMode".
 * An OpMode is a 'program' that runs in either the autonomous or the teleop period of an FTC match.
 * The names of OpModes appear on the menu of the FTC Driver Station.
 * When a selection is made from the menu, the corresponding OpMode
 * class is instantiated on the Robot Controller and executed.
 *
 * This particular OpMode just executes a basic Tank Drive Teleop for a two wheeled robot
 * It includes all the skeletal structure that all iterative OpModes contain.
 *
 * Use Android Studio to Copy this Class, and Paste it into your team's code folder with a new name.
 * Remove or comment out the @Disabled line to add this OpMode to the Driver Station OpMode list
 */

@TeleOp(name="Teach: State Example Iterative", group="Teach")
//@Disabled
public class ColorAndDistanceIterative extends OpMode
{
    // Declare OpMode members.

    // color sensor.
    NormalizedColorSensor colorSensor;
    float gain = 2.0f;
    float[] hsvValues = new float[3];

    NormalizedRGBA colors;

    // front-facing distance sensor.
    DistanceSensor front;
    final float TARGET_DIST = 6.0f;          // inches
    double currDist = 0;

    TeachBot bot;

    // the robot states.
    // IDLE - the robot is not doing anything. Push the X and A buttons to switch to MOVING_FORWARD.
    // MOVING_FORWARD - the robot is moving forward.
    // if it detects red it pauses.
    // if it is within the target distance to the front sensor, it is done and goes to IDLE.
    // PAUSED - the robot has detected red and stays paused until red is removed.
    enum BotState {IDLE, MOVING_FORWARD, PAUSED}
    BotState currState;

    GamepadManager pad1Manager;
    private ElapsedTime runtime = new ElapsedTime();

    /*
     * Code to run ONCE when the driver hits INIT
     */
    @Override
    public void init() {
        // get reference to the teaching bot.
        bot = new TeachBot(hardwareMap);

        // robot starts in idle mode.
        currState = BotState.IDLE;

        // get a reference to the color sensor.
        colorSensor = hardwareMap.get(NormalizedColorSensor.class, "color");
        colorSensor.setGain(gain);

        // get a reference to distance sensor.
        front = hardwareMap.get(DistanceSensor.class, "front");

        // use a GamepadManager object to keep track of the gamepad state.
        pad1Manager = new GamepadManager(gamepad1);

        // Tell the driver that initialization is complete.
        telemetry.addData("Status", "Initialized");
    }

    /*
     * Code to run REPEATEDLY after the driver hits INIT, but before they hit START
     */
    @Override
    public void init_loop() {
    }

    /*
     * Code to run ONCE when the driver hits START
     */
    @Override
    public void start() {
        runtime.reset();
    }

    // update the state of the robot.
    public void updateState() {
        // if IDLE, user needs to press X and A to start.
        if (currState == BotState.IDLE) {
            // are the X and A buttons pressed?
            if (pad1Manager.pressed(GamepadManager.Button.X) && pad1Manager.pressed(GamepadManager.Button.A)) {
                // switch to MOVIING_FORWARD.
                currState = BotState.MOVING_FORWARD;
            }
        } else if (currState == BotState.MOVING_FORWARD) {
            // do we detect red?  if so, pause the robot.
            if (isItRed()) {
                currState = BotState.PAUSED;
            }

            // are we close enough to the target distance?
            currDist = front.getDistance(DistanceUnit.INCH);
            if (currDist <= TARGET_DIST) {
                // switch to IDLE.
                currState = BotState.IDLE;
            }
        } else if (currState == BotState.PAUSED) {
            // if the color sensor no longer detects red, then start moving again.
            if (!isItRed()) {
                currState = BotState.MOVING_FORWARD;
            }
        }
    }

    // returns true if color sensor detects red.
    public boolean isItRed() {
        // Get the normalized colors from the sensor
        colors = colorSensor.getNormalizedColors();

        // convert to HSV values.
        Color.colorToHSV(colors.toColor(), hsvValues);

        // check for red.
        if ((hsvValues[0] < 32 || hsvValues[0] > 340)
                && (hsvValues[1] > 0.4) && (hsvValues[2] > 0.015)) {
            return true;
        } else {
            return false;
        }
    }

    public void adjustGain() {
        if (pad1Manager.justPressed(GamepadManager.Button.DPAD_UP)) {
            // increase gain.
            gain += 0.05;
            if (gain > 3) {
                gain = 3f;
            }
            colorSensor.setGain(gain);
        }
        if (pad1Manager.justPressed(GamepadManager.Button.DPAD_DOWN)) {
            // increase gain.
            gain -= 0.05;
            if (gain < 1) {
                gain = 1f;
            }
            colorSensor.setGain(gain);

        }
    }

    public void adjustRobot() {
        if(currState == BotState.IDLE || currState == BotState.PAUSED) {
            bot.stop();
        } else if (currState == BotState.MOVING_FORWARD) {
            bot.drive(0, 0.3, 0);
        }
    }

    public void doTelemetry() {

        telemetry.addData("Status", "Run Time: " + runtime.toString());

        telemetry.addData("Robot State", currState.toString());
        telemetry.addData("Gain", String.format("%.2f", gain));

        // send info to driver hub.
        telemetry.addLine()
                .addData("Hue", "%.3f", hsvValues[0])
                .addData("Saturation", "%.3f", hsvValues[1])
                .addData("Value", "%.3f", hsvValues[2]);

        telemetry.addData("Front Dist (in)",  "%.2f", currDist);
    }

    /*
     * Code to run REPEATEDLY after the driver hits START but before they hit STOP
     */
    @Override
    public void loop() {
        // update gamepad state.
        pad1Manager.poll();

        // adjust gain of color sensor.
        adjustGain();

        // update the state of the robot.
        updateState();

        // adjust robot.
        adjustRobot();

        // do telemetry.
        doTelemetry();
    }

    /*
     * Code to run ONCE after the driver hits STOP
     */
    @Override
    public void stop() {
    }
}
