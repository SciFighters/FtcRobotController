package org.firstinspires.ftc.teamcode.freight_frenzy.util.autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.freight_frenzy.study.DuckLine;
import org.firstinspires.ftc.teamcode.freight_frenzy.util.DriveClass;
import org.firstinspires.ftc.teamcode.freight_frenzy.util.HandRailClass;
import org.firstinspires.ftc.teamcode.freight_frenzy.util.autonomous.AutoFlow;

@Autonomous(group = "blue carousel")
//@Disabled
public class BlueAutoCarousel extends LinearOpMode {
	@Override
	public void runOpMode() {
		AutoFlow auto = new AutoFlow(this, AutoFlow.ALLIANCE.BLUE, AutoFlow.StartPosY.CAR, AutoFlow.Auto.FULL);

		auto.init();

		waitForStart();

		auto.run();
	}
}
