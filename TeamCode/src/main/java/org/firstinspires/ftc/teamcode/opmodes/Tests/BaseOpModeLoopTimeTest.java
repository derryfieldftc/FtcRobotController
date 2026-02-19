package org.firstinspires.ftc.teamcode.opmodes.Tests;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

@TeleOp
public class BaseOpModeLoopTimeTest extends OpMode {
    double lastLoopTime;
    double averageLoopTime;

    @Override
    public void init() {

    }

    @Override
    public void loop() {
        averageLoopTime = (averageLoopTime + (this.getRuntime() - lastLoopTime)) / 2;
        lastLoopTime = this.getRuntime();

        telemetry.addData("Loop Time", averageLoopTime);
        telemetry.update();
    }
}
