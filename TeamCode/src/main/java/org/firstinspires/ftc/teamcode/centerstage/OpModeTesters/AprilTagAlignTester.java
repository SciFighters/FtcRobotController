package org.firstinspires.ftc.teamcode.centerstage.OpModeTesters;

import android.util.Size;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.centerstage.Autonomous.AutoFlow;
import org.firstinspires.ftc.teamcode.centerstage.Systems.Arm.Arm;
import org.firstinspires.ftc.teamcode.centerstage.Systems.Camera.AprilTagDetector;
import org.firstinspires.ftc.teamcode.centerstage.Systems.DriveClass;
import org.firstinspires.ftc.teamcode.centerstage.util.ECSSystem.Robot;
import org.firstinspires.ftc.teamcode.centerstage.util.Location;
import org.firstinspires.ftc.teamcode.freight_frenzy.util.MathUtil;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;

@TeleOp
public class AprilTagAlignTester extends Robot {
    final int screenWidth = 640;
    final int screenHeight = 360;
    public int tagID = 4;
    public DriveClass drive;
    MultipleTelemetry multipleTelemetry;
    AprilTagDetector aprilTagDetector;

    @Override
    public void initRobot() {
        FtcDashboard dashboard = FtcDashboard.getInstance();
        Telemetry dashboardTelemetry = dashboard.getTelemetry();
        multipleTelemetry = new MultipleTelemetry(telemetry, dashboardTelemetry);
        this.drive = addComponent(DriveClass.class, new DriveClass(DriveClass.ROBOT.GLADOS, new Location(0, 0), DriveClass.USE_ENCODERS | DriveClass.USE_BRAKE | DriveClass.USE_DASHBOARD_FIELD, DriveClass.DriveMode.LEFT));
        aprilTagDetector = new AprilTagDetector("cam", new Size(screenWidth, screenHeight), hardwareMap, multipleTelemetry, AprilTagDetector.PortalConfiguration.DEFAULT);
    }

    @Override
    public void startRobot() {

    }

    @Override
    public void updateLoop() {
        if (gamepad1.a) {
            lockOnTag(tagID);
        } else if (gamepad1.dpad_up) {
            tagID = 5;
        } else if (gamepad1.dpad_down) {
            tagID = 4;
        } else if (gamepad1.dpad_left) {
            tagID = 6;
        }
    }

    public void lockOnTag(int tagID) {
        drive.turnTo(90, 1);
        double yOffset = 0.18;
        if (tagID == 2 || tagID == 5) {
            yOffset = 0;
        } else if (tagID == 1 || tagID == 4) {
            yOffset *= -1;
        }
        if (alliance == AutoFlow.Alliance.RED) {
            yOffset *= -1;
        }
//        drive.goToLocation(new Location(backdropLocation.x, alliance == AutoFlow.Alliance.BLUE ? backdropLocation.y + yOffset : backdropLocation.flipY().y + yOffset, 90), lowToleranceSettings);
        AprilTagDetection tag = aprilTagDetector.getSpecificTag(tagID);
        double targetHeading = alliance == AutoFlow.Alliance.BLUE ? 90 : -90;
        double leftDist = drive.getDistanceLeftSensorDistance(), rightDist = drive.getDistanceRightSensorDistance();
        double distance = Math.min(leftDist, rightDist);
        int distanceSensorIndex = leftDist == distance ? 1 : 2;
        multipleTelemetry.clearAll();

        while (tag != null && (!MathUtil.approximately(tag.rawPose.x, 0, 0.1))) // in inches
//                        || !MathUtil.approximately(distance, Arm.Position.One.distanceFromBackdrop, 0.5))) // in cm
        {
            double gainY = -0.016;
//            double gainX = 0.018 / 2;

            double deltaAngle = drive.getDeltaHeading(targetHeading);
//            double deltaX = Arm.Position.One.distanceFromBackdrop - distance;
            double deltaY = tag.rawPose.x;

//            double powerX = gainX * deltaX;
            double powerY = deltaY * gainY;
//            double turn = deltaAngle * gainX;

            multipleTelemetry.addData("DISTANCE X", tag.rawPose.x);
            multipleTelemetry.addData("DISTANCE Y", tag.rawPose.y);
            multipleTelemetry.addData("left right DISTANCE", MathUtil.approximately(tag.rawPose.x, 0, 0.1));
//            multipleTelemetry.addData("backdrop DISTANCE", MathUtil.approximately(distance, Arm.Position.One.distanceFromBackdrop, 0.5));
            multipleTelemetry.update();

            drive.setPower(0, 0, powerY);
            tag = aprilTagDetector.getSpecificTag(tagID);
//            distance = distanceSensorIndex == 1 ? drive.getDistanceLeftSensorDistance() : drive.getDistanceRightSensorDistance();
//            drive.turnTo(90, MathUtil.map(1, drive.getHeading(), 90, 0, 1));
        }
        drive.setPower(0, 0, 0);
    }
}
