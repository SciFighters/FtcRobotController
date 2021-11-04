package org.firstinspires.ftc.teamcode.freight_frenzy.study;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.ultimate_goal.AutoFlows;
import org.firstinspires.ftc.teamcode.ultimate_goal.util.BananaPipeline;
import org.opencv.core.Point;
import org.opencv.core.Rect;
import org.openftc.easyopencv.OpenCvCamera.AsyncCameraOpenListener;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.easyopencv.OpenCvWebcam;

@TeleOp
public class DuckLineTester extends LinearOpMode {

	enum ABC {A, B, C}
	public int screenWidth = 640;
	public int screenHeight = 360;

	@Override
	public void runOpMode() {
		// TODO: separate to util class
		int cameraMonitorViewID = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());

		WebcamName webcamName = hardwareMap.get(WebcamName.class, "webcam");
		OpenCvWebcam webcam = OpenCvCameraFactory.getInstance().createWebcam(webcamName, cameraMonitorViewID);

		DuckLine duckline = new DuckLine();
		webcam.setPipeline(duckline);

		webcam.openCameraDeviceAsync(new AsyncCameraOpenListener() {
			@Override
			public void onOpened() {
				webcam.startStreaming(screenWidth, screenHeight, OpenCvCameraRotation.UPRIGHT);
			}

			@Override
			public void onError(int errorCode) {
				telemetry.addData("camera initialization failed", errorCode);
				telemetry.update();
			}
		});

		waitForStart();

		ABC abc = null;

		while (opModeIsActive()) {
			Point duck = duckline.getTargetPos();
			if (duck.x <= screenWidth / 3) { // First 3rd of the screen
				abc = ABC.A;
			} else if (duck.x <= screenWidth / 3 * 2) { // Second 3rd of the screen
				abc = ABC.B;
			} else if (duck.x <= screenWidth){ // Detects on the whole screen
				abc = ABC.C;
			} else {

			}

			if(abc != null) telemetry.addData("Duck at:", abc); //if duck is on screen, adding telemetry about it's location...
			else telemetry.addData("The Duck is: ", "not found"); // if the duck isn't on screen or not found
			telemetry.update();
		}
	}
}
