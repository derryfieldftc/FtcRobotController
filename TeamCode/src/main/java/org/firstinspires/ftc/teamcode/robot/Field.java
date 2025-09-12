package org.firstinspires.ftc.teamcode.robot;

import java.util.ArrayList;

public class Field {
	public enum Ball {
		Green,
		Purple,
		None
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
	}
}
