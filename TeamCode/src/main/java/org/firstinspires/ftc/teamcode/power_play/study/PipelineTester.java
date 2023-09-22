package org.firstinspires.ftc.teamcode.power_play.study;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.power_play.util.CameraInitializer;
import org.firstinspires.ftc.teamcode.power_play.util.RodLine;
import org.firstinspires.ftc.teamcode.power_play.util.SleevePipeline;
import org.openftc.easyopencv.OpenCvWebcam;

@TeleOp
public class PipelineTester extends LinearOpMode {
	@Override
	public void runOpMode() {
		SleevePipeline sleeve = new SleevePipeline();
		RodLine rod = new RodLine().useYellow();
		OpenCvWebcam cam = CameraInitializer.initialize(this, "webcam_rod", 640, 360, rod, true);

		this.telemetry = new MultipleTelemetry(this.telemetry, FtcDashboard.getInstance().getTelemetry());

		waitForStart();

		while (opModeIsActive()) {
			telemetry.addData("sleeve location", sleeve.getParkingLocation());

			telemetry.addData("rod Offset", rod.getRodCenterOffset());
			if (rod.isRodDetected())
				telemetry.addData("rod Width", rod.getRodRect().x);

			if (gamepad1.a) cam.setPipeline(sleeve);
			if (gamepad1.b) cam.setPipeline(rod);

			telemetry.update();
		}
	}
}
