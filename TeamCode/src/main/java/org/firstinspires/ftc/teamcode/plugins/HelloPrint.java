package org.firstinspires.ftc.teamcode.plugins;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.RobotPlugin;

public class HelloPrint extends RobotPlugin {
    Telemetry telemetry;
    OpMode opMode;

    public HelloPrint(OpMode opMode) {
        this.opMode = opMode;
        this.telemetry = opMode.telemetry;
    }

    @Override
    public void start() {
        telemetry.addLine("Hello World?");
        telemetry.update();
    }
}
