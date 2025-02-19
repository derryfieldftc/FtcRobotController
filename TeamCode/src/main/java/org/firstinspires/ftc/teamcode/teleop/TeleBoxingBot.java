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


import static org.firstinspires.ftc.teamcode.binarybot.EnhancedGamepad.Button.A;
import static org.firstinspires.ftc.teamcode.binarybot.EnhancedGamepad.Button.B;
import static org.firstinspires.ftc.teamcode.binarybot.EnhancedGamepad.Button.BACK;
import static org.firstinspires.ftc.teamcode.binarybot.EnhancedGamepad.Button.DPAD_DOWN;
import static org.firstinspires.ftc.teamcode.binarybot.EnhancedGamepad.Button.DPAD_LEFT;
import static org.firstinspires.ftc.teamcode.binarybot.EnhancedGamepad.Button.DPAD_RIGHT;
import static org.firstinspires.ftc.teamcode.binarybot.EnhancedGamepad.Button.DPAD_UP;
import static org.firstinspires.ftc.teamcode.binarybot.EnhancedGamepad.Button.LEFT_BUMPER;
import static org.firstinspires.ftc.teamcode.binarybot.EnhancedGamepad.Button.LEFT_STICK;
import static org.firstinspires.ftc.teamcode.binarybot.EnhancedGamepad.Button.RIGHT_BUMPER;
import static org.firstinspires.ftc.teamcode.binarybot.EnhancedGamepad.Button.RIGHT_STICK;
import static org.firstinspires.ftc.teamcode.binarybot.EnhancedGamepad.Button.START;
import static org.firstinspires.ftc.teamcode.binarybot.EnhancedGamepad.Button.Y;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.boxingBot.boxingBot;
import org.firstinspires.ftc.teamcode.boxingBot.EnhancedGamepad;

@TeleOp(name="Boxing Bot", group="Boxing Bot")
//@Disabled
public class TeleBoxingBot extends LinearOpMode {
    private boxingBot bot;
    private EnhancedGamepad epad2 = null;
    private  EnhancedGamepad epad1 = null;
    @Override
    public void runOpMode() {
        telemetry.addData("Status", "Initialized");
        telemetry.update();

        // create a new robot object.
        bot = new boxingBot(hardwareMap);
        //get enhanced gamepad
        epad1 = new EnhancedGamepad(gamepad1);
        epad2 = new EnhancedGamepad(gamepad2);
        // wait for start command from driver hub.
        waitForStart();

        // loop until opmode is stopped.
        while (opModeIsActive()) {
            epad1.poll();
            epad2.poll();
            // get gamepad input for mecanum drive

            //drive
            bot.drive(-gamepad1.left_stick_y, gamepad1.left_stick_x, -gamepad1.right_stick_x);



        }
    }
}
