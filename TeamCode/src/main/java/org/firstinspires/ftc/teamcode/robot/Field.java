package org.firstinspires.ftc.teamcode.robot;

public class Field {
	public static Alliance alliance;
	public static Motif motif = Motif.PGP; // default
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
		 * @param hsv
		 * @return
		 */
		public static Ball getBallFromColor(float[] hsv) {
			float[] purpleHSV = {210.0403078f, 0.549924593f, 1.282462609f};
			float[] purpleHSV_STD = {17.48066683f, 0.07041663864f, 1.213211678f};
			float[] greenHSV = {162.5922055f, 0.6858531926f, 0.7229373997f};
			float[] greenHSV_STD = {3.660891619f, 0.0595187757f, 0.3784698006f};
			// I frankly could not care less about s and v

			if (purpleHSV[0] - 2 * purpleHSV_STD[0] < hsv[0] && hsv[0] < purpleHSV[0] + 2 * purpleHSV_STD[0])
				return Purple;

			if (greenHSV[0] - 2 * greenHSV_STD[0] < hsv[0] && hsv[0] < greenHSV[0] + 2 * greenHSV_STD[0])
				return Green;

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

}