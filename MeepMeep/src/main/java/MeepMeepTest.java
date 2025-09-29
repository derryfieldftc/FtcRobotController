import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;

import org.rowlandhall.meepmeep.MeepMeep;
import org.rowlandhall.meepmeep.roadrunner.DefaultBotBuilder;
import org.rowlandhall.meepmeep.roadrunner.entity.RoadRunnerBotEntity;

import java.awt.Image;
import java.io.File;
import java.io.IOException;

import javax.imageio.ImageIO;

public class MeepMeepTest {
	public static void main(String[] args) {
		System.setProperty("sun.java2d.opengl", "true"); // for performance
		MeepMeep meepMeep = new MeepMeep(800);

		RoadRunnerBotEntity bot =
				red1(meepMeep);


		Image img = null;
		try {
			img = ImageIO.read(new File("MeepMeep/src/main/res/Decode.png"));
		} catch (IOException e) {
		}

		meepMeep.setBackground(img)
				.setDarkMode(true)
				.setBackgroundAlpha(0.95f)
				.addEntity(bot.setDimensions(18, 18))
//				.addEntity(red1.setDimensions(18, 18))
//				.exportTrajectoryImage("/home/xela/blue2.png")
				.setShowFPS(true)
				.start();
	}

	// Finish later
	// Red2 is still almost perfection
	public static RoadRunnerBotEntity red1(MeepMeep meepMeep) {
		Pose2d initPose = new Pose2d(20, -57, Math.PI / 4);
		return new DefaultBotBuilder(meepMeep)
				// Set bot constraints: maxVel, maxAccel, maxAngVel, maxAngAccel, track width
				.setConstraints(50, 50, Math.toRadians(180), Math.toRadians(180), 16)
				.followTrajectorySequence(drive -> drive.trajectorySequenceBuilder(initPose)
						.waitSeconds(2) // FIRE
						.splineToConstantHeading(new Vector2d(50, -50), Math.PI / 4) // collecting human player zone balls
						.splineTo(new Vector2d(60, -55), -Math.PI / 2)
						.splineTo(new Vector2d(60, -60), -Math.PI / 2)
						.setReversed(true)
						.splineToConstantHeading(new Vector2d(15, -50), Math.PI / 2)
						.setReversed(false)
						.turn(Math.PI / 2) // FIRE
						.splineToConstantHeading(new Vector2d(41, -11), 0) // Collect row 2
						.splineToConstantHeading(new Vector2d(57, -4), 0) // lever
						.waitSeconds(1)
						.setReversed(true)
						.splineToConstantHeading(new Vector2d(10, 0), Math.PI) // Back to shootable
						.waitSeconds(2) // FIRE
						.setReversed(false)
						.splineToConstantHeading(new Vector2d(41, 12), 0)
						.splineTo(new Vector2d(47, 12), 0) // Collect row 1
						.setReversed(true)
						.splineToConstantHeading(new Vector2d(10, 0), Math.PI) // Back to shootable
						.waitSeconds(2) // FIRE
						.setReversed(false)
						.splineToConstantHeading(new Vector2d(44, -35), 0)
						.setReversed(true)
						.splineToConstantHeading(new Vector2d(20, -57), Math.PI)
						.build());
	}

	public static RoadRunnerBotEntity red2(MeepMeep meepMeep) {

		Pose2d initPose = new Pose2d(45, 50, -Math.PI / 2);
		return new DefaultBotBuilder(meepMeep)
				// Set bot constraints: maxVel, maxAccel, maxAngVel, maxAngAccel, track width
				.setConstraints(50, 50, Math.toRadians(180), Math.toRadians(180), 16)
				.followTrajectorySequence(drive -> drive.trajectorySequenceBuilder(initPose)
						.strafeTo(new Vector2d(25, 25)) // Away from goal to shootable location
						.waitSeconds(2) // FIRE
						.splineTo(new Vector2d(47, 14), 0) // Collect row 3
						.splineToConstantHeading(new Vector2d(51, 6), 0) // to lever
						.splineToConstantHeading(new Vector2d(53, 6), 0) // to lever
						.waitSeconds(1) // Lever
						.setReversed(true)
						.strafeTo(new Vector2d(15, 8)) // Back to shootable
						.waitSeconds(2)// FIRE
						.setReversed(false)
						.splineToConstantHeading(new Vector2d(38, -12), 0) // Collect row 2
						.splineToConstantHeading(new Vector2d(47, -12), 0) // Collect row 2
						.setReversed(true)
						.splineToConstantHeading(new Vector2d(15, 8), Math.PI) // Shootable once more
						.waitSeconds(2) // FIRE
						.splineToConstantHeading(new Vector2d(30, -34), 0) // Align to row 1
						.splineToConstantHeading(new Vector2d(45, -36), 0) // Collect row 1
						.setReversed(true)
						.splineToConstantHeading(new Vector2d(15, -54), Math.PI) // Back to shootable low
						.waitSeconds(2) // FIRE
						.setReversed(false)
						.splineTo(new Vector2d(50, -40), 0) // above human player zone
						.splineTo(new Vector2d(60, -65), -Math.PI / 2) // Human Player
						// Too slow D:
//						.strafeTo(new Vector2d(10, -60)) // Shootable
//						.waitSeconds(2) // FIRE
						.build());
	}
}