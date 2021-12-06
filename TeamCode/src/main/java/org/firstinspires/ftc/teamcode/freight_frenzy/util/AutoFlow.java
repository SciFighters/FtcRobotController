package org.firstinspires.ftc.teamcode.freight_frenzy.util;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.freight_frenzy.study.DuckLine;
import org.firstinspires.ftc.teamcode.ultimate_goal.AutoFlows;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.easyopencv.OpenCvWebcam;

public class AutoFlow {
	private LinearOpMode opMode; // First I declared it as OpMode now its LinearOpMode
	final double tile = 0.6;


	public enum ALLIANCE {
		BLUE,
		RED
	}

	private HandRailClass handrail = null;
	private DuckLine duckLine = null;

	int mul;

	final int screenWidth = 640;
	final int screenHeight = 360;

	public AutoFlow(LinearOpMode opMode, ALLIANCE alliance) {
		this.opMode = opMode;
		alliance = alliance;

		this.handrail = new HandRailClass(opMode);
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


	}

}
