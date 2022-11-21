package org.firstinspires.ftc.teamcode.power_play.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.freight_frenzy.util.autonomous.AutoFlow;

@Autonomous(group = "RED")
//@Disabled
public class Red_Bump_FULL extends LinearOpMode {
	org.firstinspires.ftc.teamcode.freight_frenzy.util.autonomous.AutoFlow auto = new org.firstinspires.ftc.teamcode.freight_frenzy.util.autonomous.AutoFlow(this, org.firstinspires.ftc.teamcode.freight_frenzy.util.autonomous.AutoFlow.ALLIANCE.RED, org.firstinspires.ftc.teamcode.freight_frenzy.util.autonomous.AutoFlow.StartPos.BARRIER, AutoFlow.Auto.FULL);
	@Override
	public void runOpMode() {

		auto.init();

		waitForStart();

		auto.run();

		while (opModeIsActive());
	}
}
