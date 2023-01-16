package org.firstinspires.ftc.teamcode.power_play.study;

import android.util.Log;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.power_play.util.CameraInitializer;
import org.firstinspires.ftc.teamcode.power_play.util.SleevePipeline;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.easyopencv.OpenCvWebcam;

@TeleOp
public class PipelineTester extends LinearOpMode {
	@Override
	public void runOpMode() {
		SleevePipeline pipeline = new SleevePipeline();
//		CameraInitializer.initialize(this, "webcam", 640, 360, pipeline, true);
		CameraInitializer.initialize(this, "webcam", 320, 180, pipeline, true);

		this.telemetry = new MultipleTelemetry(this.telemetry, FtcDashboard.getInstance().getTelemetry());

		waitForStart();

		while (opModeIsActive()) {
			telemetry.addData("location", pipeline.getParkingLocation());
			telemetry.update();
		}
	}
}
