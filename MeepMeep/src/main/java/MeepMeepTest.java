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
		MeepMeep meepMeep = new MeepMeep(800);

		RoadRunnerBotEntity red1 = new DefaultBotBuilder(meepMeep)
				// Set bot constraints: maxVel, maxAccel, maxAngVel, maxAngAccel, track width
				.setConstraints(50, 50, Math.toRadians(180), Math.toRadians(180), 16)
				.followTrajectorySequence(drive -> drive.trajectorySequenceBuilder(new Pose2d(-16, -60, Math.PI / 2))
						.waitSeconds(2) // FIRE
						.splineTo(new Vector2d(-35, -35), Math.PI)
						.splineTo(new Vector2d(-45, -35), Math.PI)
						.setReversed(true)
						.splineTo(new Vector2d(-16, -55), -Math.PI / 2)
						.setReversed(false)
						.waitSeconds(2) // FIRE
						.splineTo(new Vector2d(-60, -45), -Math.PI / 2)
						.splineToConstantHeading(new Vector2d(-60, -60), -Math.PI / 2)
						.strafeTo(new Vector2d(-16, -55))
						.turn(Math.PI)
						.waitSeconds(2) //FIRE
						.build());

		RoadRunnerBotEntity red2 = new DefaultBotBuilder(meepMeep)
				// Set bot constraints: maxVel, maxAccel, maxAngVel, maxAngAccel, track width
				.setConstraints(50, 50, Math.toRadians(180), Math.toRadians(180), 16)
				.followTrajectorySequence(drive -> drive.trajectorySequenceBuilder(new Pose2d(-48, 48, -Math.PI / 2))
						.waitSeconds(2) // FIRE
						.splineTo(new Vector2d(-25, 15), -3 * Math.PI / 4)
						.splineTo(new Vector2d(-42, 12), Math.PI) // Collect row 3
						.splineTo(new Vector2d(-50, 12), Math.PI)
						.setReversed(true)
						.splineTo(new Vector2d(-25, 12), 0)
						.waitSeconds(2) // FIRE
						.setReversed(false)
						.strafeTo(new Vector2d(-25, -10))
						.strafeTo(new Vector2d(-50, -10)) // Collect row 2
						.strafeTo(new Vector2d(-30, -10))
						.splineToConstantHeading(new Vector2d(-14, -55), Math.PI)
						.waitSeconds(2) // FIRE
						.splineToConstantHeading(new Vector2d(-35, -35), Math.PI)
						.strafeTo(new Vector2d(-50, -35)) // Collect row 3
						.setReversed(true)
						.splineToConstantHeading(new Vector2d(-14, -55), 0)
						.setReversed(false)
						.waitSeconds(2) // FIRE
//						.strafeTo(new Vector2d(-60, -60))
//						.strafeTo(new Vector2d(-14, -55))
						.splineTo(new Vector2d(-60, -45), -Math.PI / 2)
						.splineToConstantHeading(new Vector2d(-60, -60), -Math.PI / 2)
						.strafeTo(new Vector2d(-16, -55))
						.waitSeconds(2) //FIRE
						.build());


		Image img = null;
		try {
			img = ImageIO.read(new File("MeepMeep/src/main/res/Decode.png"));
		} catch (IOException e) {
		}

		meepMeep.setBackground(img)
				.setDarkMode(true)
				.setBackgroundAlpha(0.95f)
				.addEntity(red2.setDimensions(18, 18))
//				.addEntity(red1.setDimensions(18, 18))
				.setShowFPS(true)
				.start();
	}
}
