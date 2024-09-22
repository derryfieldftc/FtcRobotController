package org.firstinspires.ftc.teamcode.opmodes;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.OpModeGroups;
import org.firstinspires.ftc.teamcode.RobotPlugin;
import org.firstinspires.ftc.teamcode.plugins.Claw;
import org.firstinspires.ftc.teamcode.plugins.MecanumDrive;

@TeleOp(name = "ClawTestOpMode", group = OpModeGroups.TESTS)
public class ClawTestOpMode extends PluginOpMode {
        @Override
        public RobotPlugin[] initPlugins() {
			Claw claw = new Claw();
            return new RobotPlugin[]{ claw };
        }
}
