package org.firstinspires.ftc.teamcode.freight_frenzy.util.autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

@Autonomous(group = "BLUE")
@Disabled
public class Blue_Bump_FULL extends LinearOpMode {
	@Override
	public void runOpMode() {
		AutoFlow auto = new AutoFlow(this, AutoFlow.ALLIANCE.BLUE, AutoFlow.StartPos.BARRIER, AutoFlow.Auto.FULL);

		auto.init();

		waitForStart();

		auto.run();

		while (opModeIsActive());
	}
}
