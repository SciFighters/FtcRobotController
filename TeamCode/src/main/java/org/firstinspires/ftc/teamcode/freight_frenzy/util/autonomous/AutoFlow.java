package org.firstinspires.ftc.teamcode.freight_frenzy.util.autonomous;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.freight_frenzy.study.DuckLine;
import org.firstinspires.ftc.teamcode.freight_frenzy.util.DriveClass;
import org.firstinspires.ftc.teamcode.freight_frenzy.util.HandRailClass;
import org.firstinspires.ftc.teamcode.freight_frenzy.util.Location;

public class AutoFlow {
	private LinearOpMode opMode = null; // First I declared it as OpMode now its LinearOpMode
	DriveClass drive = null;
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
		SHORT,
		LONG,
		PARK,
		FULL
	}
	private HandRailClass handrail = null;
	private DuckLine duckLine = null;

	private int mul;

	final double robotLength =  0.4572;
	final int screenWidth = 640;
	final int screenHeight = 360;
	Auto auto;
	Location a_pos;
	Location b_pos;
	Location c_pos;
	Location startLocation = new Location(0.6, robotLength/2); //1.1, 0.0
	Location shippingHubLocation = new Location(0.3, 0.6); //0.6, 0.75
	Location carousel = new Location(1.0, 0.16);
	Location barrier = new Location(-1.2, 1.0);

	Location freightLocation = new Location(1.5,-0.35); // 1.5, 2.5
	Location storageLocation = new Location(0,0); //0.0, 0.0
	ALLIANCE alliance;
	Location tempStartPos = new Location(0.0, 0.6);
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

		this.drive = new DriveClass(opMode, DriveClass.ROBOT.JACCOUSE, startLocation);
		this.handrail = new HandRailClass(opMode);
	}

	public void init() {
		drive.init(opMode.hardwareMap);
		handrail.init(opMode.hardwareMap);

		handrail.searchHomeRail();

		if(startPos == StartPos.BARRIER) {
			this.startLocation.flipX();
			this.shippingHubLocation.flipX();
		}

		// TODO: separate to util class
		//Wqebcam
		int cameraMonitorViewID = opMode.hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", opMode.hardwareMap.appContext.getPackageName());

		/*WebcamName webcamName = opMode.hardwareMap.get(WebcamName.class, "webcam");
		OpenCvWebcam webcam = OpenCvCameraFactory.getInstance().createWebcam(webcamName, cameraMonitorViewID);

		this.duckLine = new DuckLine();
		webcam.setPipeline(duckLine);

		webcam.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener() {
			@Override
			public void onOpened() {
				webcam.startStreaming(screenWidth, screenHeight, OpenCvCameraRotation.UPRIGHT);
			}

			@Override
			public void onError(int errorCode) {
				opMode.telemetry.addData("camera initialization failed", errorCode);
				opMode.telemetry.update();
			}
		});
		*/
		mul = alliance.mul;

		DuckLine.SH_Levels shLevel = DuckLine.SH_Levels.Middle; //duckLine.getDuck(screenWidth);
		opMode.telemetry.addData("Init - ABC", shLevel);
		opMode.telemetry.update();
	}

	public void run() { //Autonomous starts
		//		DuckLine.ABC abc = duckLine.getDuck(screenWidth);
		DuckLine.SH_Levels shLevel = DuckLine.SH_Levels.Top; //duckLine.getDuck(screenWidth);
		opMode.telemetry.addData("Start - ABC", shLevel);
		opMode.telemetry.update();

		//DuckLine.ABC abc = duckLine.getDuck(screenWidth);
		//DuckLine.ABC abc = DuckLine.ABC.C;
		int mulPos = startPos.mul;

		opMode.telemetry.addData("goTo ShippingHub x:", shippingHubLocation.x);
		opMode.telemetry.update();

		// Go to Shipping Hub
		handrail.gotoLevel(shLevel);
		drive.goToLocation(shippingHubLocation, 1, -45.0 * startPos.mul, 0.05);

		// wait for handRail to get into position (both not busy)
		opMode.sleep(5000);
		// TODO: while (opMode.opModeIsActive() && handrail.isBusy());

		// Put the cube on shipping hub
		handrail.grabberRelease();
		this.opMode.sleep(2000);
		handrail.grabberStop();
		// retract arm.
		handrail.gotoRail(50, 0.8);

		if (startPos == StartPos.CAROUSEL) {
			// Go to carousel
			opMode.telemetry.addData("goTo Carousel Y:", carousel.y);
			opMode.telemetry.update();
			drive.goToLocation(carousel, 1, 45.0, 0.15);

			opMode.telemetry.addLine("After goto carousel");
			opMode.telemetry.update();

			handrail.carouselRun(1);
			this.opMode.sleep(2000);
			handrail.carouselStop();


			if (auto == Auto.FULL){
				drive.goTo(0.6,0.6,1,90,0.05);
				handrail.gotoHandRail(0,70,1);
				drive.goToLocation(freightLocation, 1, 90, 0.05);
			} else {
				// go to parking at storage unit
				drive.goToLocation(storageLocation, 0.8, 0, 0.02);
			}
		} else {
			// TODO: BARRIER
			//drive.goToLocation(shippingHubLocation, 1,-90 ,0.5);
			drive.turnTo(90,1);
			handrail.gotoHandRail(0,85,1);
			drive.goToLocation(freightLocation, 1, 90, 0.05);
			drive.turnTo(45,1);
			handrail.gotoHandRail(0,100,1);
			while (opMode.opModeIsActive() && handrail.isBusy());
			handrail.grabberGrab(); //takes block

//			drive.goToLocation(shippingHubLocation, 1, heading, 0.05);
//			handrail.goToABC(abc);
//			handrail.goToSH_Level(shLevel);
//			drive.goToLocation(freightLocation, 1, heading, 0.05);
		}
	}
}
