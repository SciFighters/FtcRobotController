package org.firstinspires.ftc.teamcode.freight_frenzy.util.autonomous;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.freight_frenzy.study.DuckLine;
import org.firstinspires.ftc.teamcode.freight_frenzy.util.DriveClass;
import org.firstinspires.ftc.teamcode.freight_frenzy.util.HandRailClass;
import org.firstinspires.ftc.teamcode.freight_frenzy.util.Location;
import org.firstinspires.ftc.teamcode.ultimate_goal.study.Auto;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.easyopencv.OpenCvWebcam;

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
		BuMP(-1);

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

	int mul;


	final int screenWidth = 640;
	final int screenHeight = 360;
	Auto auto;
	Location a_pos;
	Location b_pos;
	Location c_pos;
	Location shippingHub = new Location(0.6, 0.75);
	Location carousel = new Location(1.1, 0.0);
	Location Barrier;
	Location freight = new Location(1.5,2.5);
	Location parkPos = new Location(0,0);
	ALLIANCE alliance;
	Location tempStartPos = new Location(0.6,1.5);
	StartPosY startPos;

	/***
	 *
	 * @param opMode
	 * @param alliance
	 * @param startPosY
	 * @param auto
	 */
	public AutoFlow(LinearOpMode opMode, ALLIANCE alliance, StartPosY startPosY, Auto auto) {
		this.opMode = opMode;
		this.alliance = alliance;
		this.startPos = startPosY ;
		 this.auto = auto;

		this.drive = new DriveClass(opMode, DriveClass.ROBOT.JACCOUSE, statLine);
		this.handrail = new HandRailClass(opMode);

		drive.init(opMode.hardwareMap);
		handrail.init(opMode.hardwareMap);


		// TODO: separate to util class
		//Webcam
		// int cameraMonitorViewID = opMode.hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", opMode.hardwareMap.appContext.getPackageName());

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

//		DuckLine.ABC abc = duckLine.getDuck(screenWidth);
		DuckLine.SH_Levels shLevel = duckLine.getDuck(screenWidth);

		//DuckLine.ABC abc = duckLine.getDuck(screenWidth);
		//DuckLine.ABC abc = DuckLine.ABC.C;
		double heading = drive.getHeading();
		int mulPos;
		if (startPos == StartPosY.CAR){
			mulPos = 1;
		}else {
			mulPos =-1;
		}


		//Autonomous starts
		if (auto != Auto.PARK && startPosY != StartPosY.BuMP){
			drive.goToLocation(carousel,1, heading, 0.5);

			handrail.carouselRun(1);
			this.opMode.sleep(2000);
			handrail.carouselStop();

		}

		if (auto != Auto.PARK){
			drive.goToLocation(shippingHub, 1, heading, 0.5);

//			handrail.goToABC(abc);
			handrail.goToSH_Level(shLevel);
		}

		if (auto == Auto.FULL || startPosY == StartPosY.BuMP){
			drive.goToLocation(freight, 1, heading, 0.5);
			handrail.grabberGrab(); //takes block
			drive.goToLocation(shippingHub, 1, heading, 0.5);
//			handrail.goToABC(abc);
			handrail.goToSH_Level(shLevel);
			drive.goToLocation(parkPos,1, heading, 0.5);
		}else {
			drive.goToLocation(parkPos, 1, heading, 0.05);
		}


	}

}
