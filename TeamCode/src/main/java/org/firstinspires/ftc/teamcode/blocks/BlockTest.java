package org.firstinspires.ftc.teamcode.blocks;

import org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion;
import org.firstinspires.ftc.robotcore.external.ExportClassToBlocks;
import org.firstinspires.ftc.robotcore.external.ExportToBlocks;

@ExportClassToBlocks
public class BlockTest extends BlocksOpModeCompanion {
	public static enum Plugins {
		MEC,
		CLAW,
		TANK
	}
	@ExportToBlocks
	public static void addPlugin(String name, Plugins plugins) {
	}
}
