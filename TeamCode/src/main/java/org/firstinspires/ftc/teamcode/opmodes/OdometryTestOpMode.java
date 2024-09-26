package org.firstinspires.ftc.teamcode.opmodes;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.OpModeGroups;
import org.firstinspires.ftc.teamcode.RobotPlugin;
import org.firstinspires.ftc.teamcode.plugins.Claw;
import org.firstinspires.ftc.teamcode.plugins.MecanumDrive;
import org.firstinspires.ftc.teamcode.plugins.Odometry;

@TeleOp(name = "OdometryTestOpMode", group = OpModeGroups.TESTS)
public class OdometryTestOpMode extends PluginOpMode {
        @Override
        public RobotPlugin[] initPlugins() {
			Odometry odometry = new Odometry(this);
			MecanumDrive mecanumDrive = new MecanumDrive(this);
            return new RobotPlugin[]{ odometry, mecanumDrive };
        }
}
