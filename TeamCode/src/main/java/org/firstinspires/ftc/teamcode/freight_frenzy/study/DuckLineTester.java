package org.firstinspires.ftc.teamcode.freight_frenzy.study;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.freight_frenzy.util.DuckLine;
import org.openftc.easyopencv.OpenCvCamera.AsyncCameraOpenListener;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.easyopencv.OpenCvWebcam;

@TeleOp
@Disabled
public class DuckLineTester extends LinearOpMode {
	final private int screenWidth = 640;
	final private int screenHeight = 360;

	@Override
	public void runOpMode() {
		int cameraMonitorViewID = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());

		WebcamName webcamName = hardwareMap.get(WebcamName.class, "webcam");
		OpenCvWebcam webcam = OpenCvCameraFactory.getInstance().createWebcam(webcamName, cameraMonitorViewID);

		DuckLine duckline = new DuckLine();
		webcam.setPipeline(duckline);

		webcam.openCameraDeviceAsync(new AsyncCameraOpenListener() {
			@Override
			public void onOpened() {
				webcam.startStreaming(screenWidth, screenHeight, OpenCvCameraRotation.UPRIGHT); //
			}

			@Override
			public void onError(int errorCode) {
				telemetry.addData("camera initialization failed", errorCode);
				telemetry.update();
			}
		});

		telemetry.addLine("Initialized v2");
		telemetry.update();

		waitForStart();

		DuckLine.SH_Levels sh_level;

		while(opModeIsActive()) {
			sh_level = duckline.getDuck();

			if (sh_level != null)
				telemetry.addData("sh level:", sh_level.toString());
			else
				telemetry.addLine("no duck");

			telemetry.update();
		}
	}
}
