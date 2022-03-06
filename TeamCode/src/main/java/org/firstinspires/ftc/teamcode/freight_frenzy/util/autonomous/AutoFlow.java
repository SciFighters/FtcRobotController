package org.firstinspires.ftc.teamcode.freight_frenzy.util.autonomous;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.RobotLog;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
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

    public enum Auto{
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

	final double robotLength =  0.4572;
	final int screenWidth = 640;
	final int screenHeight = 360;
	Auto auto;
	// Declaring locations
	Location startLocation = new Location(0.6, robotLength/2); //1.1, 0.0
	Location shippingHubLocation = new Location(0.3, 0.6,-35); //0.6, 0.75
	Location shippingHubLocation_Pre1 = new Location(shippingHubLocation, -0.1, -0.15, 0);
	Location carouselLocation = new Location(1.29, 0.32,65); // 1.32, 0.27, 45
	Location barrier = new Location(-1.2, 1.0);

	// Freight locations
	Location freightLocation_Pre1 = new Location(0.6,0.80, 90); //previously 0.6, 0.92

	Location freightLocation_Pre2 = new Location(-0.60,0.9, 90); //previously -0.60, 0.85
	Location freightLocation_Pre3 = new Location(-0.0, 0.6, 90);
	Location freightLocation = new Location(-1.40,0.90, 90); // -1.5, 0.93
	Location freightPickup = new Location(-1.45, 0.1, 90);
	Location freightSideLocation = new Location(-1.1, 0.1, 90);
	private final Location pre_cycle = new Location(0, 0.4, 90);


	// Storage locations
	Location storageLocation_Pre1 = new Location(1.0,0.9, 90); //1.1, 0.92
	Location storageLocation = new Location(1.3,0.9, 90); //previously 1.5, 0.9

	ALLIANCE alliance;
	StartPos startPos;

	/**
	 * @param opMode
	 * @param alliance
	 * @param startPos
	 * @param auto
	 */
	public AutoFlow(LinearOpMode opMode, ALLIANCE alliance, StartPos startPos, Auto auto) {
		this.opMode = opMode;
		this.alliance = alliance;
		this.startPos = startPos;
		this.auto = auto;

		if(startPos == StartPos.BARRIER) {
			this.startLocation.flipX();
			this.shippingHubLocation.flipX();
			this.shippingHubLocation.flipAngle();
		}

		if (alliance == ALLIANCE.RED) {
			//TODO: flip all location on x axis (<location>.flipX())
			opMode.telemetry.addLine("red");
			opMode.telemetry.update();
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

			//flipping angles
			//this.freightLocation.flipAngle();
			//this.freightLocation_Pre1.flipAngle();
			//this.freightLocation_Pre2.flipAngle();
			//this.storageLocation.flipAngle();
			//this.storageLocation_Pre1.flipAngle();
			this.carouselLocation.angle = 135;
			this.shippingHubLocation.flipAngle();
			this.shippingHubLocation_Pre1.flipAngle();
			this.freightPickup.flipAngle(); // TODO: adjust it so it turns the right way
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
				opMode.telemetry.update();
			}
		});
	}

	public void init() {
		initWebcam();
		drive.init(opMode.hardwareMap);
		handrail.init(opMode.hardwareMap);

		handrail.searchHome();

		opMode.telemetry.addData("duck position", this.duckline.getDuck());
		opMode.telemetry.update();
	}

	public void run() { //Autonomous starts
		if (auto != Auto.PARK && auto != Auto.SHORT) {
			// Put the cube on the shipping hub
			DuckLine.SH_Levels shLevel = this.duckline.getDuck();
			opMode.telemetry.addData("SH Level:", shLevel);

			opMode.telemetry.addData("goTo ShippingHub x:", shippingHubLocation.x);
			opMode.telemetry.update();

			// Go to Shipping Hub
			handrail.gotoRail(50, 0.5);
			drive.goToLocation(shippingHubLocation, 1, 0.05, 0);

			// wait for handRail to get into position (both not busy)
			handrail.gotoLevel(shLevel);
			while (opMode.opModeIsActive() && handrail.isBusy());
			if(shLevel != DuckLine.SH_Levels.Top) handrail.gotoRail(75, 0.5);
			while (opMode.opModeIsActive() && handrail.isBusy());

			// Put the cube on shipping hub
			handrail.grabberRelease();
			opMode.sleep(1950); //wait for drop
			handrail.grabberStop();
			// retract arm.
			handrail.gotoRail(50, 0.4);
			while(opMode.opModeIsActive() && handrail.isBusy());
			handrail.gotoHandRail(0,80, 1);
		}

		if (startPos == StartPos.CAROUSEL && auto != Auto.PARK && auto != Auto.CYCLING) {
			// Going to carousel
			opMode.telemetry.addData("goTo Carousel Y:", carouselLocation.y);
			opMode.telemetry.update();
			drive.goToLocation(carouselLocation, 0.8, 0.05, 6); //previous tolerance 0.15
			handrail.carouselRun(0.5 * alliance.mul);
			opMode.telemetry.addLine("running carousel");
			opMode.telemetry.update();
			drive.setPower(0, 0, 0.1); //activates carousel motor
			if(alliance == ALLIANCE.RED)
				opMode.sleep(350); //sleeps (thread) for 0.2 seconds
			else
				opMode.sleep(350);
			drive.setPower(0, 0, 0); // stops strafe

			// Spinning carousel
			opMode.telemetry.addLine("After goto carousel");
			opMode.telemetry.update();
			if(alliance == ALLIANCE.BLUE) {
				opMode.sleep(2600);  //wait for carousel
			} else {
				opMode.sleep(2500);
			}
			handrail.carouselStop(); //Stopping carousel motor

			opMode.telemetry.addLine("stop carousel");
			opMode.telemetry.update();

			if (auto == Auto.FULL) {
				drive.goToLocation(freightLocation_Pre1, 1, 0.2, 0);
				handrail.gotoHandRail(0, 70, 1);
				drive.goToLocation(freightLocation_Pre2, 1, 0.2, 0); //first location
				drive.goToLocation(freightLocation, 1, 0.2, 0);
				drive.turn(180, 0.82);
				//TODO: replace if (collect or middle), with collect and implement an if to change collect in HandRailClass
				if (alliance == ALLIANCE.BLUE)
					handrail.gotoLevel(DuckLine.SH_Levels.Middle);
				else
					handrail.gotoLevel(DuckLine.SH_Levels.Collect);

			} else {
				// go to parking at storage unit
				parkStorage();
//				drive.setPower(0.45,0,0);
//				opMode.sleep(100);
//				drive.setPower(0,0,0);

			}

		} else {
			// Barrier (freight)
			if(startPos == StartPos.BARRIER) {
				drive.goToLocation(freightLocation_Pre3, 1, 0.15, 0); //first location
				drive.goToLocation(freightLocation_Pre2, 1, 0.15, 0); //first location
				drive.goToLocation(freightLocation, 1, 0.05, 0);
				// TODO: Collect freight item, moreover, place it on the shipping hub
			} else {
				parkStorage();
			}
			if(auto == Auto.CYCLING) {
				for(int i = 0; i < 3; i++) {
					if (i==2){
						cycle(true);
					} else {
						cycle(false);
					}
				}
			}
				//TODO: replace if (collect or middle), with collect and implement an if to change collect in HandRailClass
			if(auto != Auto.PARK) {
				if (alliance == ALLIANCE.BLUE)
					handrail.gotoLevel(DuckLine.SH_Levels.Collect);
				else
					handrail.gotoLevel(DuckLine.SH_Levels.Middle);
			}
		}

    }

    private void parkStorage() {
		RobotLog.d("going to storageLocation");
		drive.goToLocation(storageLocation_Pre1,1,0.02,0); //first location
		opMode.telemetry.addData("after", drive.getHeading());
		drive.goToLocation(storageLocation, 1,  0.02, 0);
		opMode.telemetry.addData("now", drive.getHeading());
	}

	private void cycle(boolean park) {
		// TODO: adjust power, tolerance and locations for cycle
		drive.goToLocation(pre_cycle, 1, 0.10, 0); // first location - pre-barrier
		drive.goToLocation(freightSideLocation, 0.5, 0.03, 0);
		handrail.gotoLevel(DuckLine.SH_Levels.Collect);
		drive.goToLocation(freightPickup, 0.5, 0.025, 0);
		//pickup loop
		//grabbing
		handrail.grabberGrab();
		opMode.sleep(1600); // TODO: adjust sleep duration
		handrail.grabberStop();
		// Going back to shipping hub
		drive.goToLocation(freightSideLocation, 0.6, 0.2, 0);
		drive.goToLocation(freightLocation_Pre2, 0.6, 0.05, 0);
		handrail.gotoLevel(DuckLine.SH_Levels.Top);
		drive.goToLocation(shippingHubLocation_Pre1, 0.75, 0.06, 0);
		drive.goToLocation(shippingHubLocation, 0.75, 0.04, 0);
		handrail.grabberRelease();
		opMode.sleep(1250);
		if(park) {
			drive.goToLocation(freightLocation, 1, 0.15, 0);
		}
	}
}
