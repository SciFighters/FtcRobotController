package org.firstinspires.ftc.teamcode.freight_frenzy.util.autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

@Autonomous(group = "blue carousel")
//@Disabled
public class BlueAutoCarouselLong extends LinearOpMode {
	AutoFlow auto;
	@Override
	public void runOpMode() throws InterruptedException {
		auto = new AutoFlow(this, AutoFlow.ALLIANCE.BLUE, AutoFlow.StartPos.CAROUSEL, AutoFlow.Auto.LONG);
		auto.init();

		waitForStart();

		auto.run();

		while (opModeIsActive());
	}
}
