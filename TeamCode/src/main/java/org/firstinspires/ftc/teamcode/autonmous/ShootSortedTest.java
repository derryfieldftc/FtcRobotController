package org.firstinspires.ftc.teamcode.autonmous;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.teamcode.autonmous.actions.Action;
import org.firstinspires.ftc.teamcode.autonmous.actions.InstantAction;
import org.firstinspires.ftc.teamcode.autonmous.actions.SequentialAction;
import org.firstinspires.ftc.teamcode.robot.Robot;

@Autonomous
@Disabled
public class ShootSortedTest extends OpMode {
    Robot robot;
    Action action;
    boolean running = true;

    @Override
    public void init() {
        robot = new Robot(this);
        action = new SequentialAction(
                robot.shootAllSorted()
        );

    }

    @Override
    public void loop() {
        robot.spindexer.updateBalls();
        if (running) {
            running = action.run();
        } else {
            stop();
        }
    }
}
