package org.firstinspires.ftc.teamcode.freight_frenzy.study;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.openftc.easyopencv.OpenCvCamera.AsyncCameraOpenListener;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.easyopencv.OpenCvWebcam;

@TeleOp
public class DuckLineTester extends LinearOpMode {
	@Override
	public void runOpMode() {
		// TODO: separate to util class
		int cameraMonitorViewID = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());

		WebcamName webcamName = hardwareMap.get(WebcamName.class, "webcam");
		OpenCvWebcam webcam = OpenCvCameraFactory.getInstance().createWebcam(webcamName, cameraMonitorViewID);

		DuckLine pipeline = new DuckLine();
		webcam.setPipeline(pipeline);

		webcam.openCameraDeviceAsync(new AsyncCameraOpenListener() {
			@Override
			public void onOpened() {
				webcam.startStreaming(640, 360, OpenCvCameraRotation.UPRIGHT);
			}

			@Override
			public void onError(int errorCode) {
				telemetry.addData("camera initialization failed", errorCode);
				telemetry.update();
			}
		});

		waitForStart();

		while (opModeIsActive()) {
			//
		}
	}
}
