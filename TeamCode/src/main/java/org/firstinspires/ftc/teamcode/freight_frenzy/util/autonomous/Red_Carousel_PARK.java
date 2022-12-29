package org.firstinspires.ftc.teamcode.freight_frenzy.util.autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

@Autonomous(group = "RED")
@Disabled
public class Red_Carousel_PARK extends LinearOpMode {
	AutoFlow auto = new AutoFlow(this, AutoFlow.ALLIANCE.RED, AutoFlow.StartPos.CAROUSEL, AutoFlow.Auto.PARK);
	@Override
	public void runOpMode() {

		auto.init();

		waitForStart();

		auto.run();

		while (opModeIsActive());
	}
}
