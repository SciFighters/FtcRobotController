package org.firstinspires.ftc.teamcode.centerstage;

import android.util.Size;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.hardware.camera.Camera;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.centerstage.util.CameraPipeline;
import org.firstinspires.ftc.teamcode.centerstage.util.DriveClass;
import org.firstinspires.ftc.teamcode.centerstage.util.Input;
import org.firstinspires.ftc.teamcode.centerstage.util.KeyCode;
import org.firstinspires.ftc.teamcode.centerstage.util.Toggle;
import org.firstinspires.ftc.teamcode.power_play.util.Location;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;
import org.opencv.core.Point;

@TeleOp(group = "Linear Opmode")
public class BasicTestRobot extends LinearOpMode {
    DriveClass drive = new DriveClass(this, DriveClass.ROBOT.JACCOUSE, new Location(-0.9, 0.4404 / 2, 180), DriveClass.USE_ENCODERS | DriveClass.USE_BRAKE, DriveClass.DriveMode.LEFT);
    boolean allowMovement = true; // Flag to control movement
    Toggle rotateToggle = new Toggle(); // toggle to check rotation fix
    double angle = 0;
    final double maxTagSize = 239;
    CameraPipeline cameraPipeline;

    @Override
    public void runOpMode() {

        drive.init(hardwareMap);
        cameraPipeline = new CameraPipeline("cam", new Size(640, 480), hardwareMap, this.telemetry);
        // april tag identification initiator
        waitForStart();

        double targetHeading = drive.getHeading();
        drive.resetOrientation(0);

        while (!isStopRequested() && opModeIsActive()) {
            rotateToggle.update(gamepad1.a);
            if (gamepad1.start) {
                if (gamepad1.x) {
                    drive.resetOrientation(0);
                    targetHeading = drive.getHeading();
                }
                continue;
            }

            final double boostK = 0.5;
            double boost = gamepad1.right_trigger * boostK + (1 - boostK);
            double y = pow(gamepad1.left_stick_y) * boost;
            double x = pow(-gamepad1.left_stick_x) * boost;
            double turn = pow(gamepad1.right_stick_x * boost);

            if (gamepad1.dpad_left) drive.setPower(0, 0, 0.1);
            else if (gamepad1.dpad_right) drive.setPower(0, 0, -0.1);
            else if (gamepad1.dpad_up) drive.setPower(-0.1, 0, 0);
            else if (gamepad1.dpad_down) drive.setPower(0.1, 0, 0);
            else {
                if (cameraPipeline.getDetections().size() > 0) { // Some tag is detected
                    drive.setPowerOriented(y, x, turn, !gamepad1.start);
                    allowMovement = true;
                    AprilTagDetection tag = cameraPipeline.getSpecificTag(5); // Get the right wall middle tag
                    if (tag == null) {
                        continue;
                    }
                    if (rotateToggle.isClicked()) {
                        if (!drive.busy)
                            drive.turn(-tag.ftcPose.yaw, 0.11);
                    }
                    cameraPipeline.printTelemetryData(tag);
                    angle = -tag.ftcPose.yaw;
                } else {
                    drive.setPowerOriented(y, x, turn, !gamepad1.start);
                    allowMovement = true;
                }
                telemetry.addData("tags detected ", (int) cameraPipeline.getDetections().size());
                telemetry.addData("Allow Movement", allowMovement);
                telemetry.update();
            }
            Input.updateControls(gamepad1, gamepad2);
        }
    }

    public double pow(double x) {
        return Math.pow(x, 2) * Math.signum(x);
    }


}
