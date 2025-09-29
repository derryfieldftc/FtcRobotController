package org.firstinspires.ftc.teamcode.robot;

import static org.firstinspires.ftc.teamcode.robot.Field.Alliance;
import static org.firstinspires.ftc.teamcode.robot.Field.Ball;
import static org.firstinspires.ftc.teamcode.robot.Field.Motif;

import com.acmerobotics.roadrunner.Vector2d;

import java.util.ArrayList;

public class Depot {
	public ArrayList<Ball> balls = new ArrayList<>(9);
	Alliance alliance;

	public Depot(Alliance alliance) {
		this.alliance = alliance;
	}

	;

	/**
	 * Returns the amount of balls in the gutter
	 *
	 * @return
	 */
	public int addBall(Ball ball) {
		if (balls.size() < 9)
			balls.add(ball);

		return balls.size();
	}

	public int motifScore(Motif motif) {
		int correctSpots = 0;

		for (int i = 0; i < 9; i++) {
			if (balls.get(i) == motif.getBall(i % 3)) {
				correctSpots++;
			}
		}

		return correctSpots;
	}

	public Tag getTag(Alliance alliance) {
		if (alliance == Alliance.Red)
			return Tag.RED;

		return Tag.BLUE;
	}

	/**
	 * Returns the positon of the Alliance Depot in relation to the field, Keep in mind the origin is the center of the field
	 *
	 * @param alliance
	 * @return
	 */
	public Vector2d getPosition(Alliance alliance) {
		if (alliance == Alliance.Red) {
			return new Vector2d(70, 70);
		}
		return new Vector2d(-70, 70);
	}

	/**
	 * Returns the position of the alliance depot in relation to the field, remeber the origin is at the center of the field.
	 *
	 * @return
	 */
	public Vector2d getPosition() {
		return getPosition(alliance);
	}
}
