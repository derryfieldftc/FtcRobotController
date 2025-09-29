package org.firstinspires.ftc.teamcode.robot;

import android.graphics.Color;

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

}