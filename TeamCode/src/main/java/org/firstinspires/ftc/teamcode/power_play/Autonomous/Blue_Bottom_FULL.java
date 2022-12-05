package org.firstinspires.ftc.teamcode.power_play.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.freight_frenzy.util.autonomous.AutoFlow;

@Autonomous(group = "BLUE")
//@Disabled
public class Blue_Bottom_FULL extends LinearOpMode {
	@Override
	public void runOpMode() {
		AutoPower auto = new AutoPower(this, AutoPower.ALLIANCE.BLUE, AutoPower.StartPos.BOTTOM, AutoPower.Auto.FULL);

		auto.init();

		waitForStart();

		auto.run();

		while (opModeIsActive());
	}
}
