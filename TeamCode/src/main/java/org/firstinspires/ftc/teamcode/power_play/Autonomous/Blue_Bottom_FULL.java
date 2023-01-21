package org.firstinspires.ftc.teamcode.power_play.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

@Autonomous(group = "BLUE")
@Disabled
public class Blue_Bottom_FULL extends LinearOpMode {
	@Override
	public void runOpMode() {
		AutoFlow auto = new org.firstinspires.ftc.teamcode.power_play.Autonomous.AutoFlow(this, AutoFlow.StartPos.LEFT, AutoFlow.Auto.FULL);

		auto.init();

		waitForStart();

		auto.run();
	}
}
