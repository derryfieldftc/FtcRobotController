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
import org.firstinspires.ftc.teamcode.binarybot.*;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.RobotLog;

import java.io.File;
import java.io.FileNotFoundException;
import java.nio.charset.StandardCharsets;
import java.util.ArrayList;
import java.util.Scanner;

@TeleOp(name="Drivetrain Test", group="Drivetrain")
//@Disabled
public class DrivetrainTest extends OpMode
{
    // Declare OpMode members.
    private ElapsedTime runtime = new ElapsedTime();

    private Drivetrain train = null;

    private EnhancedGamepad enhanced1 = null;
    private boolean turboMode = false;
    private boolean autoMode = false;
    private static final double SNAIL_FACTOR = 0.3;

    ArrayList<Pose>waypoints;
    Pose initPose;

    public void toggleAuto() {
        if (autoMode) {
            autoMode = false;
        } else {
            autoMode = true;
        }
    }

    /*
     * Code to run ONCE when the driver hits INIT
     */
    @Override
    public void init() {
        // create drive train with initial pose of 0, 0, and 90 degrees.
        //initPose = new Pose(0 * 12 * 2.54, 0 * 12 * 2.54, 0);
        initPose = new Pose (60.96,0.0,1.57);
        train = new Drivetrain(hardwareMap, this);
        train.setPose(initPose);
        enhanced1 = new EnhancedGamepad(gamepad1);
        waypoints = importWaypoints("/sdcard/FIRST/waypoints.csv");

        // Tell the driver that initialization is complete.
        telemetry.addData("Status", "Initialized");
    }

    public ArrayList<Pose> importWaypoints(String path) {
        ArrayList<Pose> list = new ArrayList<>();
        File file = new File(path);
        // make sure you can read UTF 8 characters.
        try(Scanner in = new Scanner(file, StandardCharsets.UTF_8.name());) {
            // read line by line.
            while(in.hasNextLine()) {
                // read in token by token.
                String line = in.nextLine();
                Scanner data = new Scanner(line);
                data.useDelimiter("[\\s,]+");
                String token = data.next().trim();
                // remove any non alphanumeric characters.
                token = token.replaceAll("[^a-zA-Z0-9\\.\\-]", "");
                double x = Double.parseDouble(token);
                RobotLog.d(String.format("TIE: x = %.2f", x));
                token = data.next().trim();
                token = token.replaceAll("[^a-zA-Z0-9\\.\\-]", "");
                double y = Double.parseDouble(token);
                RobotLog.d(String.format("TIE: y = %.2f", y));
                token = data.next().trim();
                token = token.replaceAll("[^a-zA-Z0-9\\.\\-]", "");
                double theta = Double.parseDouble(token);
                Pose pose = new Pose(x, y, theta);
                RobotLog.d(String.format("TIE: theta = %.2f", theta));
                list.add(pose);
            }
        } catch (FileNotFoundException e) {
            RobotLog.e("DS_Robotics: import_poses() failed.");
            RobotLog.e("DS_Robotics: " + e.getMessage());
        }
        return list;
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

    /*
     * Code to run REPEATEDLY after the driver hits START but before they hit STOP
     */
    @Override
    public void loop() {
        // refresh pose (position and heading) of the robot using odometry.
        train.refreshPose();

        // poll the enhanced gamepad to update states.
        enhanced1.poll();

        // reset odometry system?
        if (enhanced1.justPressed(EnhancedGamepad.Button.X)) {
            // reset the odometry system.
            train.resetOdometry();

            // reset init pose.
            train.setPose(initPose);
        }

        // enable turbo mode?
        if (enhanced1.justPressed(EnhancedGamepad.Button.BACK)) {
            // toggle between snail mode (slower) and regular mode
            turboMode = !turboMode;
        }

        // chalk down?
        if (enhanced1.justPressed(EnhancedGamepad.Button.A)) {
            train.toggleChalk();
        }

        if (enhanced1.justPressed(EnhancedGamepad.Button.DPAD_UP)){
            boolean val = train.getMotorCorrectionEnabled();
            val = !val;
            train.setMotorCorrectionEnabled(val);
        }

        telemetry.addData("Instructions", "Dpad down toggle auto");
        if (enhanced1.justPressed(EnhancedGamepad.Button.DPAD_DOWN)) {
            toggleAuto();
        }
        telemetry.addData("Auto Mode", autoMode);
        // display current waypoint.
        telemetry.addData("Current Waypoint", train.getCurrentWaypoint());

        if(autoMode) {
            // navigate to waypoints autonomously.
            if (train.waypoint != null) {
                // we currently have a valid waypoint to navigate to.
                if (train.applyCorrection()) {
                    // if applyCorrection() returns true, then we have arrived at the waypoint.
                    train.stop();
                    RobotLog.d("TIE: made it to waypoint. Clearing waypoint...");
                    train.clearWaypoint();
                    RobotLog.d("TIE: cleared!");
                }
            } else {
                // get a new waypoint, if available.
                if (waypoints.size() > 0) {
                    train.waypoint = waypoints.get(0);
                    waypoints.remove(0);
                }
            }
        } else {
            // also get driver input and drive robot.
            double drive = turboMode ? -gamepad1.left_stick_y : -SNAIL_FACTOR * gamepad1.left_stick_y;
            double strafe = turboMode ? gamepad1.left_stick_x : SNAIL_FACTOR * gamepad1.left_stick_x;
            double turn  =  turboMode ? -gamepad1.right_stick_x : -SNAIL_FACTOR * gamepad1.right_stick_x;
            train.drive(drive, strafe, turn);
        }

        // Show the elapsed game time and wheel power.
        telemetry.addData("Turbo Mode", turboMode);
        telemetry.addData("Motor Correction Enabled", train.getMotorCorrectionEnabled());

        // encoder data
        telemetry.addData("x", train.pose.x);
        telemetry.addData("y", train.pose.y);
        telemetry.addData("theta", train.pose.theta);

        // update telemetry.
        telemetry.update();
    }

    /*
     * Code to run ONCE after the driver hits STOP
     */
    @Override
    public void stop() {
        train.close_log();
    }
}