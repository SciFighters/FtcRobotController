package org.firstinspires.ftc.teamcode.power_play.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

@Autonomous(group = "BLUE")
//@Disabled
public class TestAuto extends LinearOpMode {
	@Override
	public void runOpMode() {
		AutoFlow auto = new AutoFlow(this, AutoFlow.ALLIANCE.BLUE, AutoFlow.StartPos.back, AutoFlow.Auto.FULL);

		auto.init();

		waitForStart();

		auto.placeCones();
	}
}
