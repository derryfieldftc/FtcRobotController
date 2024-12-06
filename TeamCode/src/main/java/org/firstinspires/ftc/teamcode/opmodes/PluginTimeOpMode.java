package org.firstinspires.ftc.teamcode.opmodes;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.OpModeGroups;
import org.firstinspires.ftc.teamcode.RobotPlugin;
import org.firstinspires.ftc.teamcode.plugins.PluginTime;
import org.firstinspires.ftc.teamcode.plugins.Delayer;

@TeleOp(name = "PluginTimeTest", group = OpModeGroups.TESTS)
public class PluginTimeOpMode extends PluginOpMode {

    @Override
    public RobotPlugin[] initPlugins() {
        RobotPlugin pluginTime = new PluginTime(this);
        RobotPlugin delayer = new Delayer(this);
        return new RobotPlugin[]{ pluginTime, delayer };
    }
}