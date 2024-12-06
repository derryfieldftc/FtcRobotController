package org.firstinspires.ftc.teamcode.plugins;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.teamcode.RobotPlugin;

public class Delayer extends RobotPlugin {
    OpMode opMode;

    public Delayer(OpMode opMode) {
        this.opMode = opMode;
    }

    public void loop() {
        try {
            Thread.sleep((int)(Math.random() * 100));
        } catch (Exception ignored) {

        }
    }
}
