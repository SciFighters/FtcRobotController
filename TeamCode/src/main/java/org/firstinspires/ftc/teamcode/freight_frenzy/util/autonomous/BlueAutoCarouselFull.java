package org.firstinspires.ftc.teamcode.freight_frenzy.util.autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

@Autonomous(group = "blue carousel")
//@Disabled
public class BlueAutoCarouselFull extends LinearOpMode {
	@Override
	public void runOpMode() {
		AutoFlow auto = new AutoFlow(this, AutoFlow.ALLIANCE.BLUE, AutoFlow.StartPos.CAROUSEL, AutoFlow.Auto.FULL);

		auto.init();

		waitForStart();

		auto.run();

		while (opModeIsActive());
	}
}
