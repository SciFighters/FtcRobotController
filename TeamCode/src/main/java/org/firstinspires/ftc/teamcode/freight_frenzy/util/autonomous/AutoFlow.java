package org.firstinspires.ftc.teamcode.freight_frenzy.util.autonomous;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.freight_frenzy.util.DuckLine;
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

	final double robotLength =  0.4572;
	final int screenWidth = 640;
	final int screenHeight = 360;
	Auto auto;
	Location a_pos;
	Location b_pos;
	Location c_pos;
	Location startLocation = new Location(0.6, robotLength/2); //1.1, 0.0
	Location shippingHubLocation = new Location(0.3, 0.6); //0.6, 0.75
	Location carousel = new Location(1.32, 0.27);
	Location barrier = new Location(-1.2, 1.0);

	Location freightLocation = new Location(-1.5,0.8); // 1.5, 2.5
	Location storageLocation = new Location(1.5,0.9); //0.0, 0.0
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

		if(startPos == StartPos.BARRIER) {
			this.startLocation.flipX();
			this.shippingHubLocation.flipX();
		}

		this.drive = new DriveClass(opMode, DriveClass.ROBOT.JACCOUSE, startLocation);
		this.handrail = new HandRailClass(opMode);
	}

	public void init() {
		drive.init(opMode.hardwareMap);
		handrail.init(opMode.hardwareMap);

		handrail.searchHomeRail();


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
		opMode.telemetry.addData("goTo ShippingHub x:", shippingHubLocation.x);
		opMode.telemetry.update();

		// Go to Shipping Hub
		handrail.gotoRail(50, 0.5);
		drive.goToLocation(shippingHubLocation, 1, -45.0 * startPos.mul, 0.05, 0);

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
			// Go to carousel
			opMode.telemetry.addData("goTo Carousel Y:", carousel.y);
			opMode.telemetry.update();
			handrail.carouselRun(0.5);
			drive.goToLocation(carousel, 1, 45.0, 0.15, 6);
			//drive.setPower(0,0,0.4);
			//opMode.sleep(3000);

			opMode.telemetry.addLine("After goto carousel");
			opMode.telemetry.addLine("Kagan is a duck");
			opMode.telemetry.update();

			opMode.telemetry.addLine("running carousel");
			opMode.telemetry.addLine("Kagan is a duck");
			opMode.telemetry.update();
			opMode.sleep(2300);  //wait for carusel
			handrail.carouselStop();

			opMode.telemetry.addLine("stop carousel");
			opMode.telemetry.addLine("Kagan is a duck");
			opMode.telemetry.update();

			if (auto == Auto.FULL){
				drive.goTo(0.6,0.80,1,90,0.15, 0);
				handrail.gotoHandRail(0,70,1);
				drive.goTo(-0.60,0.80,1,90,0.15,0); //first location
				drive.goToLocation(freightLocation, 1, 90, 0.05, 0);
				handrail.gotoLevel(DuckLine.SH_Levels.Collect);
			} else {
				// go to parking at storage unit
				drive.goTo(1.1,0.9,1,90,0.05,0); //first location
				opMode.telemetry.addData("after", drive.getHeading());
				drive.goToLocation(storageLocation, 0.8, 90, 0.02, 0);
				opMode.telemetry.addData("now", drive.getHeading());

			}
		} else {
			// TODO: BARRIER
			//drive.goToLocation(shippingHubLocation, 1,-90 ,0.2, 0);
			//drive.turnTo(90,1);
			//handrail.gotoHandRail(0,85,1);
			drive.goTo(-0.60,0.80,1,90,0.15,0); //first location
			drive.goToLocation(freightLocation, 1, 90, 0.05, 0);
			handrail.gotoLevel(DuckLine.SH_Levels.Collect);
			drive.goTo(-1.5,0.7,0.7,135,0.05,0);
			//drive.turnTo(45,1);
			//handrail.gotoHandRail(0,100,1);
			//while (opMode.opModeIsActive() && handrail.isBusy());
		//	handrail.grabberGrab(); //takes block
		}
	}
}
