package org.firstinspires.ftc.teamcode.freight_frenzy.util.autonomous;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

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
	private LinearOpMode opMode = null; // First I declared it as OpMode now its LinearOpMode

	private DriveClass drive = null;
	private HandRailClass handrail = null;
	private DuckLine duckline = null;

	final double tile = 0.6;


	public enum ALLIANCE {
		BLUE(1),
		RED(-1);

		int mul;
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
		FULL(4);

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
	Location shippingHubLocation = new Location(0.3, 0.6); //0.6, 0.75
	Location carouselLocation = new Location(1.32, 0.27);
	Location barrier = new Location(-1.2, 1.0);

	// Freight locations
	Location freightLocation = new Location(-1.5,0.85); // 1.5, 2.5
	Location freightLocation_Pre1 = new Location(0.6,0.85); //previously 0.6, 0.80
	Location freightLocation_Pre2 = new Location(-0.60,0.80);
	// Storage locations
	Location storageLocation = new Location(1.5,0.92); //previously 1.5, 0.9
	Location storageLocation_Pre1 = new Location(1.1,0.92); //1.1, 0.9

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
		}

		if (alliance == ALLIANCE.RED){
			//TODO: flip all location on x axis (<location>.flipX())
			opMode.telemetry.addLine("red");
			opMode.telemetry.update();
			this.startLocation.flipX();
			this.shippingHubLocation.flipX();
			this.startLocation.flipX();
			this.shippingHubLocation.flipX();
			this.carouselLocation.flipX();
			this.barrier.flipX();
			this.freightLocation.flipX();
			this.freightLocation_Pre2.flipX();
			this.storageLocation.flipX();
			this.storageLocation_Pre1.flipX();
			this.freightLocation_Pre1.flipX();
		}

		this.drive = new DriveClass(opMode, DriveClass.ROBOT.JACCOUSE, startLocation);
		this.handrail = new HandRailClass(opMode);
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
	}

	public void run() { //Autonomous starts
		//drive.goTo(0,0,1,0,0.03,0);

		DuckLine.SH_Levels shLevel = this.duckline.getDuck();
		opMode.telemetry.addData("SH Level:", shLevel);

		opMode.telemetry.addData("goTo ShippingHub x:", shippingHubLocation.x);
		opMode.telemetry.update();

		// Go to Shipping Hub
		handrail.gotoRail(50, 0.5);
		drive.goToLocation(shippingHubLocation, 1, -45 * alliance.mul * startPos.mul, 0.05, 0);

		// wait for handRail to get into position (both not busy)
		handrail.gotoLevel(shLevel);
		while (opMode.opModeIsActive() && handrail.isBusy());
		handrail.gotoRail(100, 0.5);
		while (opMode.opModeIsActive() && handrail.isBusy());

		// Put the cube on shipping hub
		handrail.grabberRelease();
		opMode.sleep(2000); //wait for drop
		handrail.gotoRail(0, 0.4);
		handrail.grabberStop();
		// retract arm.
		handrail.gotoHand(70, 1);

		if (startPos == StartPos.CAROUSEL) {
			double carouselHeading = (alliance == ALLIANCE.BLUE) ? 45 : 135;
			// Going to carousel
			opMode.telemetry.addData("goTo Carousel Y:", carouselLocation.y);
			opMode.telemetry.update();
			drive.goToLocation(carouselLocation, 1, carouselHeading, 0.05, 6); //previous tolerance 0.15
			handrail.carouselRun(0.5 * alliance.mul);
			opMode.telemetry.addLine("running carousel");
			opMode.telemetry.update();
			drive.setPower(0,0,0.1); //activates carousel motor
			opMode.sleep(200); //sleeps (thread) for 0.2 seconds
			drive.setPower(0,0,0); // stops strafe

			// Spinning carousel
			opMode.telemetry.addLine("After goto carousel");
			opMode.telemetry.update();
			opMode.sleep(2300);  //wait for carousel
			handrail.carouselStop(); //Stopping carousel motor

			opMode.telemetry.addLine("stop carousel");
			opMode.telemetry.update();

			if (auto == Auto.FULL){
				drive.goToLocation(freightLocation_Pre1,1,90 * alliance.mul,0.15, 0);
				handrail.gotoHandRail(0,70,1);
				drive.goToLocation(freightLocation_Pre2,1,90 * alliance.mul,0.15,0); //first location
				drive.goToLocation(freightLocation, 1, 90 * alliance.mul, 0.05, 0);
				handrail.gotoLevel(DuckLine.SH_Levels.Collect);
			} else {
				// go to parking at storage unit
				drive.goToLocation(storageLocation_Pre1,1,90 * alliance.mul,0.15,0); //first location
				opMode.telemetry.addData("after", drive.getHeading());
				drive.goToLocation(storageLocation, 0.8, 90 * alliance.mul, 0.02, 0);
				opMode.telemetry.addData("now", drive.getHeading());
				drive.setPower(0.45,0,0);
				opMode.sleep(500);
				drive.setPower(0,0,0);
			}
		} else {
			// Barrier (freight)
			drive.goToLocation(freightLocation_Pre2,1,90 * alliance.mul,0.15,0); //first location
			drive.goToLocation(freightLocation, 1, 90 * alliance.mul, 0.05, 0);
			handrail.gotoLevel(DuckLine.SH_Levels.Collect);
			// TODO: Collect freight item, moreover, place it on the shipping hub
		}
    }
}
