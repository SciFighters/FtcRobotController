package org.firstinspires.ftc.teamcode.freight_frenzy.study;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.freight_frenzy.util.HandRailClass;

public class isBusyTest extends LinearOpMode {

	HandRailClass handrail = new HandRailClass(this);

	@Override
	public void runOpMode() {
		telemetry.addData("Status", "Initialized");
		telemetry.update();

		handrail.init(hardwareMap);

		waitForStart();



	}

	private void runAct() {
		this.handrail.searchHome();
		//this.handrail.searchHomeHand();
		try {
			Thread.sleep(500);
		} catch(Exception e) {
			e.printStackTrace();
			telemetry.addData("can't sleep (thread)", "  I'm not sleeping");
		}

		//checks isBUsy when using gotoHandRail
		this.handrail.gotoHandRail(75, 75, 0.6);
		while(handrail.isBusy());
		this.telemetry.addData("done goto handrail: ", "true");
		this.telemetry.update();

		try {
			Thread.sleep(500);
		} catch(Exception e) {
			e.printStackTrace();
			telemetry.addData("can't sleep (thread)", "  I'm not sleeping");
		}

		//checks isBusy when using gotoRail
		this.handrail.gotoRail(25,0.6);
		while(handrail.isBusy());
		this.telemetry.addData("done goto rail: ", "true");
		this.telemetry.update();



	}
}
