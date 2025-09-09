package org.firstinspires.ftc.teamcode.robot;

/**
 * A way to represent each tag and its id for this season
 */
public enum Tag {
	BLUE (20),
	GPP (21),
	PGP (22),
	PPG (23),
	RED (24);

	public final int id;

	// You actually cannot use a constructor for an Enum
	Tag(int id) {
		this.id = id;
	}
	public int id() {
		return this.id;
	}
}
