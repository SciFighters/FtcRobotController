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

		int multiplier;
		ALLIANCE(int multiplier) {
			this.multiplier = multiplier;
		}
	}
	public enum StartPos {
		CAROUSEL(1),
		BARRIER(-1);

		int multiplier;
		StartPos(int multiplier) {
			this.multiplier = multiplier;
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


	final int screenWidth = 640;
	final int screenHeight = 360;
	Auto auto;
	Location a_pos;
	Location b_pos;
	Location c_pos;
	Location startCarousel = new Location(0.6, 0.4572/2); //1.1, 0.0
	Location startBarrier = new Location(-0.6, 0.4572/2); //1.1, 0.0
	Location shippingHub = new Location(0.3, 0.6); //0.6, 0.75
	Location carousel = new Location(1.0, 0.16);
	Location barrier = new Location(-1.2, 1.0);

	Location freight = new Location(1.5,-0.35); // 1.5, 2.5
	Location parkPos = new Location(0,0); //0.0, 0.0
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

		Location startingPosition = startPos == StartPos.CAROUSEL ? startCarousel : startBarrier;
		this.drive = new DriveClass(opMode, DriveClass.ROBOT.JACCOUSE, startingPosition);
		this.handrail = new HandRailClass(opMode);
	}

	public void init() {
		drive.init(opMode.hardwareMap);
		handrail.init(opMode.hardwareMap);

		handrail.searchHomeRail();

		// TODO: separate to util class
		//Webcam
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
		mul = alliance.multiplier;

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
		int mulPos = startPos.multiplier;

		double heading = drive.getHeading();

		opMode.telemetry.addData("goTo ShippingHub Y:", shippingHub.y);
		opMode.telemetry.update();
		handrail.goToSH_Level(shLevel);
		drive.goToLocation(shippingHub, 1, -45.0, 0.05);

		handrail.grabberRelease();
		this.opMode.sleep(200);
		drive.drive(-0.2, 0, 0.8, drive.getHeading(), false);
		handrail.grabberStop();

		handrail.gotoRail(50, 0.8);

		opMode.telemetry.addData("goTo Carousel Y:", carousel.y);
		opMode.telemetry.update();
		drive.goToLocation(carousel, 1, 45.0, 0.15);

		opMode.telemetry.addLine("After goto carousel");
		opMode.telemetry.update();

		handrail.carouselRun(1);
		this.opMode.sleep(2000);
		handrail.carouselStop();

		if (auto == Auto.FULL || this.startPos == StartPos.BARRIER){
			drive.goToLocation(freight, 1, heading, 0.05);
			//TODO: change handrail to a position which is reasonable for grabbing a freight item
			handrail.grabberGrab(); //takes block
			drive.goToLocation(shippingHub, 1, heading, 0.05);
//			handrail.goToABC(abc);
			handrail.goToSH_Level(shLevel);
			drive.goToLocation(parkPos,1, heading, 0.05);
		} else {
			drive.goToLocation(parkPos, 1, heading, 0.05);
		}
	}
}
