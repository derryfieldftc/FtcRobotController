package org.firstinspires.ftc.teamcode.blocks;

import org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion;
import org.firstinspires.ftc.robotcore.external.ExportToBlocks;

public class BlockTest extends BlocksOpModeCompanion {
	@ExportToBlocks
	public static void sayHai() {
		telemetry.addLine("Hai");
	}
}
