package org.firstinspires.ftc.teamcode.freight_frenzy.util;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.teamcode.freight_frenzy.util.HandRailClass;
import org.firstinspires.ftc.teamcode.ultimate_goal.AutoFlows;

public class AutoFlow {
	private LinearOpMode opMode; // First I declared it as OpMode now its LinearOpMode
	final double tile = 0.6;

	public enum ALLIANCE {
		BLUE,
		RED,
	}

	enum ABC { A, B, C }

	private HandRailClass handrail = null;

	public AutoFlow (LinearOpMode OpMode, ALLIANCE alliance){
		this.opMode = OpMode;
		alliance = alliance;

		this.handrail = new HandRailClass(OpMode);

	}

}
