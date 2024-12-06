package org.firstinspires.ftc.teamcode.opmodes;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.OpModeGroups;
import org.firstinspires.ftc.teamcode.RobotPlugin;
import org.firstinspires.ftc.teamcode.plugins.HelloPrint;

@TeleOp(name = "Ali's Hello World", group = OpModeGroups.TESTS)
public class HelloPrintOpMode extends PluginOpMode {

    @Override
    public RobotPlugin[] initPlugins() {
        RobotPlugin helloPrint = new HelloPrint(this);
        return new RobotPlugin[]{ helloPrint };
    }
}
