package org.firstinspires.ftc.teamcode.robot;

import android.graphics.Color;

import com.acmerobotics.roadrunner.Vector2d;

import java.util.ArrayList;

public class Field {
	public static Alliance alliance;
	public static Motif motif;
//	public final Vector2d bottom_left;
//	public final Vector2d bottom_right;

	/**
	 * A standard way to represent the type of ball in any place, Unknown is used when it is known
	 * that there is a ball, but the color could not be determined
	 */
	public enum Ball {
		Green,
		Purple,
		Unknown,
		None;

		/**
		 * Gets which type of ball there is based on an argb value
		 *
		 * @param argb
		 * @return
		 */
		public static Ball getBallFromColor(int argb) {
			int red, green, blue, alpha;
			alpha = Color.alpha(argb);
			red = Color.red(argb);
			green = Color.green(argb);
			blue = Color.blue(argb);
			//TODO! finish this
			return None;
		}
	}

	public enum Alliance {
		Red,
		Blue
	}

	public enum Motif {
		PPG,
		PGP,
		GPP;

		public Ball getBall(int position) {
			if (this == PPG) {
				if (position == 0)
					return Ball.Purple;
				if (position == 1)
					return Ball.Purple;
				if (position == 2)
					return Ball.Green;
			} else if (this == PGP) {
				if (position == 0)
					return Ball.Purple;
				if (position == 1)
					return Ball.Green;
				if (position == 2)
					return Ball.Purple;
			} else if (this == GPP) {
				if (position == 0)
					return Ball.Green;
				if (position == 1)
					return Ball.Purple;
				if (position == 2)
					return Ball.Purple;
			}
			return Ball.None;
		}
	}

	public class Depot {
		public ArrayList<Ball> balls = new ArrayList<>(9);

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
				return new Vector2d(60, 60);
			}
			return new Vector2d(-60, 60);
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
}