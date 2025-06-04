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

import java.util.ArrayList;

@TeleOp(name="Drivetrain Test", group="Drivetrain")
//@Disabled
public class DrivetrainTest extends OpMode
{
    // Declare OpMode members.
    private ElapsedTime runtime = new ElapsedTime();

    private Drivetrain train = null;

    private EnhancedGamepad enhanced1 = null;
    private boolean turboMode = false;
    private static final double SNAIL_FACTOR = 0.3;

    ArrayList<Pose>waypoints;
    Pose initPose;

    /*
     * Code to run ONCE when the driver hits INIT
     */
    @Override
    public void init() {
        // create drive train with initial pose of 0, 0, and 90 degrees.
        initPose = new Pose(0 * 12 * 2.54, 0 * 12 * 2.54, 0);
        train = new Drivetrain(hardwareMap, this);
        train.setPose(initPose);
        enhanced1 = new EnhancedGamepad(gamepad1);
        waypoints = getWaypoints();

        // Tell the driver that initialization is complete.
        telemetry.addData("Status", "Initialized");
    }

    public ArrayList<Pose> getWaypoints() {
        ArrayList<Pose> list = new ArrayList<>();


        list.add(new Pose(100, 0, 0));
        list.add(new Pose(0, 0, 0));

//        double s = 8.0;
//        list.add(new Pose(s * 12 * 2.54, 0 * 12 * 2.54, 0));
//        list.add(new Pose(s * 12 * 2.54, 0 * 12 * 2.54, Math.PI / 2.0));
//        list.add(new Pose(s * 12 * 2.54, s * 12 * 2.54, Math.PI / 2.0));
//        list.add(new Pose(s * 12 * 2.54, s * 12 * 2.54, Math.PI / 1.0));
//        list.add(new Pose(0 * 12 * 2.54, s * 12 * 2.54, Math.PI / 1.0));
//        list.add(new Pose(0 * 12 * 2.54, s * 12 * 2.54, 3.0 * Math.PI / 2.0));
//        list.add(new Pose(0 * 12 * 2.54, 0 * 12 * 2.54, 3.0 * Math.PI / 2.0));
//        list.add(new Pose(0 * 12 * 2.54, 0 * 12 * 2.54, 4.0 * Math.PI / 2.0));

//        list.add(new Pose(2 * 12 * 2.54, 6 * 12 * 2.54, Math.PI / 2.0));
//        list.add(new Pose(6 * 12 * 2.54, 2 * 12 * 2.54, 0));
//        list.add(new Pose(2 * 12 * 2.54, 2 * 12 * 2.54, 0));

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

        // are their waypoints available?
        if (waypoints.size() > 0) {
            telemetry.addData("Waypoints", "Press right bumper to load next waypoint");
            if (enhanced1.justPressed(EnhancedGamepad.Button.RIGHT_BUMPER)) {
                Pose nextWaypoint = waypoints.get(0);
                waypoints.remove(0);
                train.setWaypoint(nextWaypoint);
            }
        }

        // clear current waypoint?
        if (train.waypoint != null) {
            telemetry.addData("Clear", "Press left bumper and B to clear current waypoint");
            if (enhanced1.justPressed(EnhancedGamepad.Button.LEFT_BUMPER) && enhanced1.justPressed(EnhancedGamepad.Button.B)) {
                train.clearWaypoint();
                telemetry.addData("Clear", "Waypoint cleared.");
            }
        }

        // display current waypoint.
        telemetry.addData("Current Waypoint", train.getCurrentWaypoint());

        // apply correction to auto navigate to current waypoint.
        if (train.applyCorrection()) {
            // if applyCorrection() returns true, then we are at the waypoint.
            train.stop();
            train.clearWaypoint();
        }

        // also get driver input and drive robot.
        double drive = turboMode ? -gamepad1.left_stick_y : -SNAIL_FACTOR * gamepad1.left_stick_y;
        double strafe = turboMode ? gamepad1.left_stick_x : SNAIL_FACTOR * gamepad1.left_stick_x;
        double turn  =  turboMode ? -gamepad1.right_stick_x : -SNAIL_FACTOR * gamepad1.right_stick_x;
        train.drive(drive, strafe, turn);

        // Show the elapsed game time and wheel power.
        telemetry.addData("Status", "Run Time: " + runtime.toString());
        telemetry.addData("Turbo Mode", turboMode);
        telemetry.addData("Motor Correction Enabled", train.getMotorCorrectionEnabled());

        telemetry.addData("drive", drive);
        telemetry.addData("strafe", strafe);
        telemetry.addData("turn", turn);

        telemetry.addData("pos left", train.encoderLeft.getCurrentPosition());
        telemetry.addData("pos right", train.encoderRight.getCurrentPosition());
        telemetry.addData("pos aux", train.encoderAux.getCurrentPosition());

        // encoder data
//        telemetry.addData("encoder left", train.encoderLeft.getCurrentPosition());
//        telemetry.addData("encoder right", train.encoderRight.getCurrentPosition());
//        telemetry.addData("encoder horiz", train.encoderSide.getCurrentPosition());
        telemetry.addData("x (in)", train.pose.x / 2.54);
        telemetry.addData("y (in)", train.pose.y / 2.54);
        telemetry.addData("theta (deg)", train.pose.theta / Math.PI * 180.0);

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