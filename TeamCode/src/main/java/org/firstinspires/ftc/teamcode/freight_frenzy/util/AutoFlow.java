package org.firstinspires.ftc.teamcode.freight_frenzy.util;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.freight_frenzy.study.DuckLine;
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
		BLUE,
		RED
	}
	public enum StartPosY {
		CAR,
		BuMP
	}

	boolean shortened = false;

	private HandRailClass handrail = null;
	private DuckLine duckLine = null;

	int mul;

	final int screenWidth = 640;
	final int screenHeight = 360;

	Location a_pos;
	Location b_pos;
	Location c_pos;
	Location shippingHub;
	Location carousel;
	Location bump;
	Location  freight;
	Location parkPos;


	public AutoFlow(LinearOpMode opMode, ALLIANCE alliance, StartPosY startPosY, boolean shortened) {
		this.opMode = opMode;
		alliance = alliance;
		StartPosY startPos = startPosY ;
		this.shortened = shortened;

		this.drive = new DriveClass(opMode, DriveClass.ROBOT.JACCOUSE, statLine);
		this.handrail = new HandRailClass(opMode);

		drive.init(opMode.hardwareMap);
		handrail.init(opMode.hardwareMap);


		// TODO: separate to util class
		int cameraMonitorViewID = opMode.hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", opMode.hardwareMap.appContext.getPackageName());

		WebcamName webcamName = opMode.hardwareMap.get(WebcamName.class, "webcam");
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

		if (alliance == ALLIANCE.BLUE) {
			mul = 1;// blue
		} else {
			mul = -1;// red
		}

		DuckLine.ABC abc = duckLine.getDuck(screenWidth);

		double heading = drive.getHeading();
		int mulPos;
		if (startPos == StartPosY.CAR){
			mulPos = 1;
		}else {
			mulPos =-1;
		}

		drive.goToLocation(carousel,1, heading, 0.5);

		handrail.carouselRun(1);
		this.opMode.sleep(2000);
		handrail.carouselStop();

		drive.goToLocation(shippingHub, 1, heading, 0.5);

		handrail.goToABC(abc);

		if (shortened == false){
			drive.goToLocation(freight, 1, heading, 0.5);
			drive.goToLocation(shippingHub, 1, heading, 0.5);
			handrail.goToABC(abc);
			drive.goToLocation(parkPos,1, heading, 0.5);
		}else {
			drive.goToLocation(parkPos, 1, heading, 0.05);
		}
	}

}
