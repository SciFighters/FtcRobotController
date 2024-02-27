package org.firstinspires.ftc.teamcode.centerstage.OpModeTesters;

import android.util.Size;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.centerstage.Systems.Camera.AprilTagDetector;
import org.firstinspires.ftc.teamcode.centerstage.util.ECSSystem.Robot;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;


@TeleOp
public class AprilTagTester extends Robot {
    final int screenWidth = 640;
    final int screenHeight = 360;
    AprilTagDetector aprilTagDetector;

    @Override
    public void initRobot() {
        aprilTagDetector = new AprilTagDetector("cam", new Size(screenWidth, screenHeight), this.hardwareMap, this.telemetry, AprilTagDetector.PortalConfiguration.DEFAULT);
    }

    @Override
    public void startRobot() {
    }

    @Override
    public void updateLoop() {
        AprilTagDetection tag = aprilTagDetector.getDetections().get(0);
        if (tag != null) {
            telemetry.addData("TAG ID", tag.id);
            telemetry.addData("TAG X", tag.rawPose.x);
            telemetry.addData("TAG Y", tag.rawPose.y);
            telemetry.addData("TAG Z", tag.rawPose.z);
            telemetry.update();
        }
    }

    @Override
    public void onStop() {
        aprilTagDetector.stop();
    }
}