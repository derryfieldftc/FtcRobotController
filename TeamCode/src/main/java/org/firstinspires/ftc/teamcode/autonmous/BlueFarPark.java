package org.firstinspires.ftc.teamcode.autonmous;

import static com.qualcomm.robotcore.util.RobotLog.d;

import com.bylazar.telemetry.PanelsTelemetry;
import com.bylazar.telemetry.TelemetryManager;
import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathChain;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.teamcode.autonmous.actions.Action;
import org.firstinspires.ftc.teamcode.autonmous.actions.FollowPathAction;
import org.firstinspires.ftc.teamcode.autonmous.actions.ParallelAction;
import org.firstinspires.ftc.teamcode.autonmous.actions.SequentialAction;
import org.firstinspires.ftc.teamcode.pedro.Constants;
import org.firstinspires.ftc.teamcode.robot.Depot;
import org.firstinspires.ftc.teamcode.robot.Field;
import org.firstinspires.ftc.teamcode.robot.Robot;
import org.firstinspires.ftc.teamcode.robot.TurretPose;

@Autonomous
public class BlueFarPark extends OpMode {
    public Follower follower; // Pedro Pathing follower instance
    private int pathState; // Current autonomous path state (state machine)
    private Paths paths; // Paths defined in the Paths class
    boolean completed = true;
    boolean resetting = true;
    Action action;
    Robot robot;
    private TelemetryManager panelsTelemetry;

    @Override
    public void init() {
        robot = new Robot(this);

        robot = new Robot(this).setTurretPose(new TurretPose(new Pose(46, 9, Math.toRadians(135)).mirror(), 0));

        panelsTelemetry = PanelsTelemetry.INSTANCE.getTelemetry();

        follower = Constants.createFollower(hardwareMap);
        follower.setStartingPose(new Pose(56, 8, Math.toRadians(90)));

        paths = new Paths(follower); // Build paths
        action = new ParallelAction(

                new FollowPathAction(follower, paths.Park),
                robot.turret.trackTarget(Depot.getPosition(Field.Alliance.Blue), follower.poseTracker.getLocalizer()),
                new Action() {
                    @Override
                    public boolean run() {
                        robot.turret.savePosition();
                        return true;
                    }
                }
        );
        while (robot.spindexer.resetPosition().run());
    }

    @Override
    public void init_loop() {
        d("AHM searching");
        telemetry.addLine("Searching");
        robot.getMotif(follower.getPoseTracker().getLocalizer()).run();
        d("AHM FOUND " + Field.motif);
        telemetry.addData("motif", Field.motif);
        telemetry.update();
    }

    @Override
    public void loop() {
        follower.update(); // Update Pedro Pathing
        if (completed)
            completed = action.run();

    }

    public static class Paths {
        public PathChain Park;

        public Paths(Follower follower) {
            Park = follower.pathBuilder().addPath(
                            new BezierLine(
                                    new Pose(56.000, 8.000),

                                    new Pose(35.770, 11.678)
                            )
                    ).setConstantHeadingInterpolation(Math.toRadians(90))

                    .build();
        }
    }

}
