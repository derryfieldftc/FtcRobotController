package org.firstinspires.ftc.teamcode.robot;

import com.pedropathing.geometry.Pose;
import com.pedropathing.math.Vector;

public class Obelisk {
	public enum Motif {
		PPG (23),
		PGP (22),
		GPP (21);

		final int id;

		Motif(int id) {
			this.id = id;
		}

		public Field.Ball getBall(int position) {
			if (this == PPG) {
				if (position == 0)
					return Field.Ball.Purple;
				if (position == 1)
					return Field.Ball.Purple;
				if (position == 2)
					return Field.Ball.Green;
			} else if (this == PGP) {
				if (position == 0)
					return Field.Ball.Purple;
				if (position == 1)
					return Field.Ball.Green;
				if (position == 2)
					return Field.Ball.Purple;
			} else if (this == GPP) {
				if (position == 0)
					return Field.Ball.Green;
				if (position == 1)
					return Field.Ball.Purple;
				if (position == 2)
					return Field.Ball.Purple;
			}
			return Field.Ball.None;
		}
	}

	public static Vector getObeliskPosition() {
		return new Vector(new Pose(72, 150));
	}
}
