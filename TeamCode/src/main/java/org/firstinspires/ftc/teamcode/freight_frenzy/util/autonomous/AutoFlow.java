package org.firstinspires.ftc.teamcode.freight_frenzy.util.autonomous;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.RobotLog;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.freight_frenzy.study.PotTest;
import org.firstinspires.ftc.teamcode.freight_frenzy.util.DuckLine;
import org.firstinspires.ftc.teamcode.freight_frenzy.util.DriveClass;
import org.firstinspires.ftc.teamcode.freight_frenzy.util.HandRailClass;
import org.firstinspires.ftc.teamcode.freight_frenzy.util.Location;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.easyopencv.OpenCvWebcam;

public class AutoFlow {
	private LinearOpMode opMode = null; // First I declared it as OpMode now it's LinearOpMode

	private DriveClass drive = null;
	private HandRailClass handrail = null;
	private DuckLine duckline = null;

	final double tile = 0.6;


	public enum ALLIANCE {
		BLUE(1),
		RED(-1);

		public int mul;

		ALLIANCE(int mul) {
			this.mul = mul;
		}
	}

	public enum StartPos {
		CAROUSEL(1),
		BARRIER(-1);

		int mul;

		StartPos(int mul) {
			this.mul = mul;
		}
	}

	public enum Auto {
		SHORT(1),
		LONG(2),
		PARK(3),
		FULL(4),
		CYCLING(5);

		public int value;

		Auto(int value) {
			this.value = value;
		}
	}

	final double robotLength = 0.4572;
	final int screenWidth = 640;
	final int screenHeight = 360;
	Auto auto;
	// Declaring locations
	Location startLocation = new Location(0.6, robotLength / 2); //1.1, 0.0
	Location shippingHubLocation = new Location(0.3, 0.58, -30); //0.6, 0.75
	Location shippingHubLocation_Pre1 = new Location(shippingHubLocation, -0.2, 0, 70);
	Location carouselLocation = new Location(1.2, 0.27, 65); // 1.32, 0.27, 45
	Location barrier = new Location(-1.2, 1.0);

	// Freight locations

	Location freightLocation_Pre1 = new Location(0.6, 0.75, -90); //previously 0.6, 0.92
	Location freightLocation_Pre2 = new Location(-0.60, 0.65, -90); //previously -0.60, 0.85
	Location freightLocation_Pre3 = new Location(0.15, 0.55, -90);
	Location freightLocation = new Location(-1.40, 0.61, -90); // -1.5, 0.93
	Location freightPickup = new Location(-1.3, 0.16, -90);
	Location freightSideLocation = new Location(-0.6, 0.1, -90);
	private final Location pre_cycle = new Location(-0.5, 0.16, -90);
	Location pre_fullPickup = new Location(freightLocation, 0.2, 0.2, -90);

	// Storage locations
	Location storageLocation_Pre1 = new Location(1.2, 0.75, -90); //1.1, 0.92
	Location storageLocation = new Location(1.3, 0.9, -90); //previously 1.5, 0.9

	ALLIANCE alliance;
	StartPos startPos;


	public AutoFlow(LinearOpMode opMode, ALLIANCE alliance, StartPos startPos, Auto auto) {
		this.opMode = opMode;
		this.alliance = alliance;
		this.startPos = startPos;
		this.auto = auto;

		if (startPos == StartPos.BARRIER) {
			this.startLocation.flipX();
			this.shippingHubLocation.flipX();
			this.shippingHubLocation_Pre1.flipX();
			this.shippingHubLocation.flipAngle();
		}

		if (alliance == ALLIANCE.RED) {
			//TODO: flip all location on x axis (<location>.flipX())
			opMode.telemetry.addLine("RED");

			this.startLocation.flipX();
			this.shippingHubLocation_Pre1.flipX();
			this.shippingHubLocation.flipX();
			this.carouselLocation.flipX();
			this.barrier.flipX();
			this.freightLocation.flipX();
			this.freightLocation_Pre2.flipX();
			this.storageLocation.flipX();
			this.storageLocation_Pre1.flipX();
			this.freightLocation_Pre1.flipX();
			this.freightLocation_Pre3.flipX();
			this.freightSideLocation.flipX();
			this.freightPickup.flipX();
			this.pre_fullPickup.flipX();
			this.pre_cycle.flipX();

			//flipping angles
			this.freightLocation.flipAngle();
			this.freightLocation_Pre1.flipAngle();
			this.freightLocation_Pre2.flipAngle();
			this.freightLocation_Pre3.flipAngle();
			this.storageLocation.flipAngle();
			this.storageLocation_Pre1.flipAngle();
			this.carouselLocation.angle = 135;
			this.shippingHubLocation.flipAngle();
			this.shippingHubLocation_Pre1.flipAngle();
			this.pre_fullPickup.flipAngle();
			this.freightPickup.flipAngle(); // TODO: adjust it so it turns the right way
			this.pre_cycle.flipAngle();
		}

		this.drive = new DriveClass(opMode, DriveClass.ROBOT.JACCOUSE, startLocation).useEncoders();
		this.handrail = new HandRailClass(opMode, this.alliance);
	}

	void initWebcam() {
		int cameraMonitorViewID = opMode.hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", opMode.hardwareMap.appContext.getPackageName());

		WebcamName webcamName = opMode.hardwareMap.get(WebcamName.class, "webcam");
		OpenCvWebcam webcam = OpenCvCameraFactory.getInstance().createWebcam(webcamName, cameraMonitorViewID);

		this.duckline = new DuckLine();
		webcam.setPipeline(this.duckline);

		webcam.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener() {
			@Override
			public void onOpened() {
				webcam.startStreaming(screenWidth, screenHeight, OpenCvCameraRotation.UPRIGHT); //
			}

			@Override
			public void onError(int errorCode) {
				opMode.telemetry.addData("camera initialization failed", errorCode);
			}
		});
	}

	public void init() {
		initWebcam();
		drive.init(opMode.hardwareMap);
		handrail.init(opMode.hardwareMap);
		handrail.setCappingPos(0);
		handrail.searchHome();

		opMode.telemetry.addData("DUCK Position", this.duckline.getDuck());
		opMode.telemetry.update();

	}

	public void run() { //Autonomous starts
		if (auto != Auto.PARK && auto != Auto.SHORT) {
			// Put the cube on the shipping hub
			DuckLine.SH_Levels shLevel = this.duckline.getDuck();
			opMode.telemetry.addData("SH Level:", shLevel);

			// Go to Shipping Hub
			handrail.gotoRail(30, 0.5);
			drive.goToLocation(shippingHubLocation, 1, 0.04, 0);
			// wait for handRail to get into position (both not busy)
			handrail.gotoLevel(shLevel);
			double time2= opMode.time;
			while (opMode.opModeIsActive() && handrail.isBusy() && (opMode.time>time2+0.5)) ;
			handrail.grabberRelease();

			if (shLevel != DuckLine.SH_Levels.Top) {
				handrail.gotoRail(75, 1);
				//while (opMode.opModeIsActive() && handrail.isBusy()) ;
			}
			// Put the cube on shipping hub
			opMode.sleep(1500); //wait for drop
			handrail.grabberStop();
			// retract arm.
			handrail.gotoRail(50, 0.4);
			while (opMode.opModeIsActive() && handrail.isBusy()) ;
			handrail.gotoHandRail(0, 80, 1);
		}

		if (startPos == StartPos.CAROUSEL && auto != Auto.PARK && auto != Auto.CYCLING) {
			// Going to carousel
			opMode.telemetry.addData("goTo Carousel Y:", carouselLocation.y);
			opMode.telemetry.update();
			drive.goToLocation(carouselLocation, 0.6, 0.05, 6); //previous tolerance 0.15
			handrail.carouselRun(0.6 * alliance.mul);
			opMode.telemetry.addLine("running carousel");
			opMode.telemetry.update();
			opMode.sleep(300);
			drive.setPower(0, 0, 0.19); //activates carousel motor\555
			if (alliance == ALLIANCE.RED)
				opMode.sleep(450); //sleeps (thread) for 0.2 seconds
			else
				opMode.sleep(450);
			drive.stopPower(); // stops strafe

			// Spinning carousel
			opMode.telemetry.addLine("After goto carousel");
			opMode.telemetry.update();
			if (alliance == ALLIANCE.BLUE) {
				opMode.sleep(2600);  //wait for carousel
			} else {
				opMode.sleep(2500);
			}
			handrail.carouselStop(); //Stopping carousel motor

			opMode.telemetry.addLine("stop carousel");
			opMode.telemetry.update();

			if (auto == Auto.FULL) {
				drive.goToLocation(freightLocation_Pre1, 1, 0.2, 0);
				drive.goToLocation(freightLocation_Pre2, 1, 0.2, 0); //first location
				drive.goToLocation(freightLocation, 1, 0.05, 0);

			} else {
				// go to parking at storage unit
				parkStorage();
//				drive.setPower(0.45,0,0);
//				opMode.sleep(100);
//				drive.setPower(0,0,0);

			}

		} else {
			// Barrier (freight)
			if (startPos == StartPos.BARRIER && auto != Auto.CYCLING) {
				drive.goToLocation(freightLocation_Pre3, 1, 0.15, 0); //first location
				drive.goToLocation(freightLocation_Pre2, 1, 0.15, 0); //first location
				drive.goToLocation(freightLocation, 1, 0.05, 0);
				// TODO: Collect freight item, moreover, place it on the shipping hub
			} else if (auto != Auto.CYCLING) {
				parkStorage();
			}
			if (auto == Auto.CYCLING) {
				boolean check = true;
				check = cycle(false, check, 0);
				check = cycle(false, check, 1);
				check = cycle(true, check, 2);

			}
		}

		//handrail.gotoRail(0, 1);
	}

	private void parkStorage() {
		RobotLog.d("going to storageLocation");
		drive.goToLocation(storageLocation_Pre1, 1, 0.02, 0); //first location
		opMode.telemetry.addData("after", drive.getHeading());
		drive.goToLocation(storageLocation, 1, 0.02, 0);
		opMode.telemetry.addData("now", drive.getHeading());
	}

	private boolean cycle(boolean park, boolean check, double round) {
		if (!check) return false;
		opMode.telemetry.addLine("cycle");
		opMode.telemetry.update();
		handrail.gotoRail(100, 1);
		drive.goToLocation(pre_cycle, 1, 0.1, 0); // first location - pre-barrier
		drive.turnTo(pre_cycle.angle, 0.5);
		handrail.gotoLevel(DuckLine.SH_Levels.CollectAuto);
		handrail.grabberGrab();
		//TOUCH SWITCH CHECKER FAZE
		double firstRoundX = -1.3;
		double reachTheFreightCalc = (firstRoundX - round * 0.05) * alliance.mul;
		drive.goTo(reachTheFreightCalc, 0.16, 1, this.freightLocation.angle, 0.05, 0);
		double timer = opMode.time;
		while(opMode.opModeIsActive() && !handrail.freightIn() && (opMode.time < timer + 1.5));
		if (!handrail.freightIn()) {
			drive.drive(0.05, 0, 1, this.freightLocation.angle, false);
			while (opMode.opModeIsActive() && !handrail.freightIn()) {
				if (opMode.time > timer + 4) {
					return false;
				}
			}
		}
		// Going back to shipping hub
		if (!park) {
			handrail.gotoRail(100, 1);
			drive.goToLocation(pre_cycle, 1, 0.05, 0);
			handrail.gotoLevel(DuckLine.SH_Levels.Top);
			handrail.grabberStop();
			//drive.goToLocation(shippingHubLocation_Pre1, 0.75, 0.06, 0);
			opMode.sleep(100);
			drive.goToLocation(shippingHubLocation, 1, 0.05, 0);
			handrail.grabberRelease();
			opMode.sleep(1000);
			handrail.grabberStop();
		} else {
			handrail.gotoHandRail(0, 80, 1);
		}
		return true;
	}
}

/* kfar yona practice:
fixing red side active:
AutoFlow: locations, sleep, disable last grab in full
DriveClass: gain formula, tolerance
HandRailClass: hand tolerance
init (jaccouse -> AutoFlow): searchHome() */