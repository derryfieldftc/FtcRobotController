package org.firstinspires.ftc.teamcode.plugins;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.RobotPlugin;

public class PluginTime extends RobotPlugin {
    Telemetry telemetry;
    OpMode opMode;

    Double currTime;
    Double lastTime = 0.0;
    Double changeTime;

    public PluginTime(OpMode opMode) {
        this.opMode = opMode;
        this.telemetry = opMode.telemetry;
    }

    @Override
    public void loop() {
        currTime = opMode.getRuntime();
        changeTime = currTime - lastTime;
        lastTime = currTime;
        telemetry.addData("Time since last update: ", changeTime);
        // telemetry.update();
    }

}
