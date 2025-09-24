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

		RoadRunnerBotEntity myBot = new DefaultBotBuilder(meepMeep)
				// Set bot constraints: maxVel, maxAccel, maxAngVel, maxAngAccel, track width
				.setConstraints(50, 50, Math.toRadians(180), Math.toRadians(180), 16)
				.followTrajectorySequence(drive -> drive.trajectorySequenceBuilder(new Pose2d(0, 0, 0))
						.strafeTo(new Vector2d(0, 24))
						.strafeTo(new Vector2d(24, 24))
						.strafeTo(new Vector2d(24, 0))
						.strafeTo(new Vector2d(0, 0))
						.build());


		Image img = null;
		try {
			img = ImageIO.read(new File("MeepMeep/src/main/res/Decode.png"));
		} catch (IOException e) {
		}

		meepMeep.setBackground(img)
				.setDarkMode(true)
				.setBackgroundAlpha(0.95f)
				.addEntity(myBot)
				.setShowFPS(true)
				.start();
	}
}
