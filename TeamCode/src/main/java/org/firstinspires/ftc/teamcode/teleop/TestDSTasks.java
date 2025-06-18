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

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.RobotLog;

import org.firstinspires.ftc.teamcode.binarybot.DSLog;
import org.firstinspires.ftc.teamcode.binarybot.PID;
import org.firstinspires.ftc.teamcode.binarybot.RobotData;
import org.firstinspires.ftc.teamcode.binarybot.Task;
import org.firstinspires.ftc.teamcode.binarybot.TaskList;

import java.util.ArrayList;

@TeleOp(name="TestDSTasks", group="Drivetrain")
//@Disabled
public class TestDSTasks extends OpMode
{
    // Declare OpMode members.
    private ElapsedTime runtime = new ElapsedTime();
    private DSLog ds_log;
    private double start_time;

    private PID pid = null;

    TaskList tasks = null;
    /*
     * Code to run ONCE when the driver hits INIT
     */
    @Override
    public void init() {
        String path = "/sdcard/FIRST/dslog.txt";
        ds_log = new DSLog(path);
        RobotLog.d("TIE: ds_log created (%s)", path);
        telemetry.addData("Status", "Initialized");
        pid = PID.importPID("/sdcard/FIRST/samplePID.txt");
        RobotLog.d("TIE: p = %.6f, i = %.6f, d = %.6f, threshold = %.6f", pid.kp, pid.ki, pid.kd, pid.threshold);

        PID.exportPID("/sdcard/FIRST/outPID.txt", pid);
        RobotLog.d("TIE: wrote PID values to outPID.txt");

        // import tasks.
        tasks = new TaskList();
        tasks.importTasks("/sdcard/FIRST/dsTasks.txt");
        RobotLog.d("TIE: wrote PID values to outPID.txt");

        // display.
        RobotLog.d("TIE: " + tasks.toString());
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
        start_time = runtime.milliseconds();
    }

    /*
     * Code to run REPEATEDLY after the driver hits START but before they hit STOP
     */
    @Override
    public void loop() {

        double curr_time = runtime.milliseconds();
        if (curr_time - start_time > 5000) {
            String msg = runtime.toString();
            ds_log.log(msg);
            start_time = curr_time;
            RobotLog.d("TIE: " + msg);
        }

        // Show the elapsed game time and wheel power.
        telemetry.addData("Status", "Run Time: " + runtime.toString());
    }

    /*
     * Code to run ONCE after the driver hits STOP
     */
    @Override
    public void stop() {
        ds_log.close();
        RobotLog.d("TIE: ds_log closed.");
    }
}
