package org.firstinspires.ftc.teamcode.power_play.util;

import android.util.Log;

import com.acmerobotics.dashboard.FtcDashboard;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.easyopencv.OpenCvPipeline;
import org.openftc.easyopencv.OpenCvWebcam;

public class CameraInitializer {
	public static void initialize(OpMode opmode, String cameraName, int width, int height, OpenCvPipeline pipeline, boolean streamToDashboard) {
		int cameraMonitorViewID = opmode.hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", opmode.hardwareMap.appContext.getPackageName());

		WebcamName webcamName = opmode.hardwareMap.get(WebcamName.class, cameraName);
		OpenCvWebcam webcam = OpenCvCameraFactory.getInstance().createWebcam(webcamName, cameraMonitorViewID);

		webcam.setPipeline(pipeline);

		webcam.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener() {
			@Override
			public void onOpened() {
				webcam.startStreaming(width, height, OpenCvCameraRotation.UPRIGHT);
				FtcDashboard.getInstance().startCameraStream(webcam, 0);
			}

			@Override
			public void onError(int errorCode) {
				opmode.telemetry.addData("camera initialization failed", errorCode);
				FtcDashboard.getInstance().getTelemetry().addLine("camera initialization failed: " + errorCode);
				Log.e("Sci", "camera initialization failed: " + errorCode);
			}
		});
	}
}
