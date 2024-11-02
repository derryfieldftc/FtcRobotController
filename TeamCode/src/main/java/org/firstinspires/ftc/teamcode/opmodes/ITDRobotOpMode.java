package org.firstinspires.ftc.teamcode.opmodes;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.OpModeGroups;
import org.firstinspires.ftc.teamcode.RobotPlugin;
import org.firstinspires.ftc.teamcode.plugins.ITDClaw;
import org.firstinspires.ftc.teamcode.plugins.MecanumDrive;

@TeleOp(name = "Into the DEEP", group = OpModeGroups.COMPETITION)
public class ITDRobotOpMode extends PluginOpMode {
    @Override
    public RobotPlugin[] initPlugins() {
        return new RobotPlugin[] {
                new MecanumDrive(this).gamepad(gamepad1),
                new ITDClaw(this, gamepad2)
        };
    }
}
