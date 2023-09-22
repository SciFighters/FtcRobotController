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
	/**
	 *
	 * Supported resolutions:
	 * [640x480],
	 * [160x90],
	 * [160x120],
	 * [176x144],
	 * [320x180],
	 * [320x240],
	 * [352x288],
	 * [432x240],
	 * [640x360],
	 * [800x448],
	 * [800x600],
	 * [864x480],
	 * [960x720],
	 * [1024x576],
	 * [1280x720],
	 * [1600x896],
	 * [1920x1080],
	 * [2304x1296],
	 * [2304x1536]
	 *
	 * */
	public static OpenCvWebcam initialize(OpMode opmode, String cameraName, int width, int height, OpenCvPipeline pipeline, boolean streamToScreen) {
		WebcamName webcamName = opmode.hardwareMap.get(WebcamName.class, cameraName);

		OpenCvWebcam webcam;
		if (streamToScreen) {
			int cameraMonitorViewID = opmode.hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", opmode.hardwareMap.appContext.getPackageName());
			webcam = OpenCvCameraFactory.getInstance().createWebcam(webcamName, cameraMonitorViewID);
		} else {
			webcam = OpenCvCameraFactory.getInstance().createWebcam(webcamName);
		}

		webcam.setPipeline(pipeline);

		webcam.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener() {
			@Override
			public void onOpened() {
				webcam.startStreaming(width, height, OpenCvCameraRotation.UPSIDE_DOWN);
			}

			@Override
			public void onError(int errorCode) {
				opmode.telemetry.addData("camera initialization failed", errorCode);
				FtcDashboard.getInstance().getTelemetry().addLine("camera initialization failed: " + errorCode);
				Log.e("Sci", "camera initialization failed: " + errorCode);
			}
		});

		return webcam;
	}
}
