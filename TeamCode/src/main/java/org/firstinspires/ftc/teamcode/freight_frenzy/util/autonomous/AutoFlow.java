package org.firstinspires.ftc.teamcode.freight_frenzy.util.autonomous;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.freight_frenzy.study.DuckLine;
import org.firstinspires.ftc.teamcode.freight_frenzy.util.DriveClass;
import org.firstinspires.ftc.teamcode.freight_frenzy.util.HandRailClass;
import org.firstinspires.ftc.teamcode.freight_frenzy.util.Location;

public class AutoFlow {
	private LinearOpMode opMode = null; // First I declared it as OpMode now its LinearOpMode
	private Location statLine = new Location(0,0);
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
	public enum StartPosY {
		CAR(1),
		Barrier(-1);

		int multiplier;
		StartPosY(int multiplier) {
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
	Location shippingHub = new Location(0.3, 0.4); //0.6, 0.75
	Location carousel = new Location(1.1, 0.16); //1.1, 0.0
	Location Barrier;
	Location freight = new Location(1.5,-0.35); // 1.5, 2.5
	Location parkPos = new Location(0,0); //0.0, 0.0
	ALLIANCE alliance;
	Location tempStartPos = new Location(0.0, 0.6);
	StartPosY startPos;

	/**
	 * @param opMode
	 * @param alliance
	 * @param startPosY
	 * @param auto
	 */
	public AutoFlow(LinearOpMode opMode, ALLIANCE alliance, StartPosY startPosY, Auto auto) {
		this.opMode = opMode;
		this.alliance = alliance;
		this.startPos = startPosY;
		this.auto = auto;

		this.drive = new DriveClass(opMode, DriveClass.ROBOT.JACCOUSE, statLine);
		this.handrail = new HandRailClass(opMode);
	}

	public void init() {
		drive.init(opMode.hardwareMap);
		handrail.init(opMode.hardwareMap);


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
		DuckLine.SH_Levels shLevel = DuckLine.SH_Levels.Middle; //duckLine.getDuck(screenWidth);
		opMode.telemetry.addData("Start - ABC", shLevel);
		opMode.telemetry.update();

		//DuckLine.ABC abc = duckLine.getDuck(screenWidth);
		//DuckLine.ABC abc = DuckLine.ABC.C;
		int mulPos;
		if (startPos == StartPosY.CAR) {
			mulPos = 1;
		} else {
			mulPos = -1;
		}

		double heading = drive.getHeading();

		if (auto != Auto.PARK) { // TODO: && this.startPos != StartPosY.Barrier
			opMode.telemetry.addData("goTo ShippingHub Y:", shippingHub.y);
			opMode.telemetry.update();
			drive.goToLocation(shippingHub, 1, 45.0, 0.05);
			//			handrail.goToABC(abc);
			handrail.goToSH_Level(shLevel);
		}


		if (auto != Auto.PARK){
			opMode.telemetry.addData("goTo Carousel Y:", carousel.y);
			opMode.telemetry.update();
			drive.goToLocation(carousel, 1, -45.0, 0.05);

			handrail.carouselRun(1);
			this.opMode.sleep(2000);
			handrail.carouselStop();
		}

		if (auto == Auto.FULL || this.startPos == StartPosY.Barrier){
			drive.goToLocation(freight, 1, heading, 0.05);
			//TODO: change handrail to a position which is reasonable for grabbing a freight item
			handrail.grabberGrab(); //takes block
			drive.goToLocation(shippingHub, 1, heading, 0.05);
//			handrail.goToABC(abc);
			handrail.goToSH_Level(shLevel);
			drive.goToLocation(parkPos,1, heading, 0.05);
		}else {
			drive.goToLocation(parkPos, 1, heading, 0.05);
		}
	}
}
