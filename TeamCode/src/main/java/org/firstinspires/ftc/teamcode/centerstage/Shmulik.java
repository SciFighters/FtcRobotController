package org.firstinspires.ftc.teamcode.centerstage;

import android.util.Size;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.centerstage.Systems.Arm.Arm;
import org.firstinspires.ftc.teamcode.centerstage.Systems.Camera.CameraPipeline;
import org.firstinspires.ftc.teamcode.centerstage.Systems.DriveClass;
import org.firstinspires.ftc.teamcode.centerstage.util.Input.Input;
import org.firstinspires.ftc.teamcode.centerstage.util.Input.Toggle;
import org.firstinspires.ftc.teamcode.centerstage.util.Input.UpdateAutomatically;
import org.firstinspires.ftc.teamcode.power_play.util.Location;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;

@Config
@TeleOp(group = "CENTERSTAGE")
public class Shmulik extends LinearOpMode {
    FtcDashboard dashboard;
    DriveClass drive = new DriveClass(this, DriveClass.ROBOT.JACCOUSE, new Location(-0.9, 0.4404 / 2, 180), DriveClass.USE_ENCODERS | DriveClass.USE_BRAKE, DriveClass.DriveMode.LEFT);
    boolean allowMovement = true; // Flag to control movement
    double angle = 0;
    //region Camera
    final double maxTagSize = 239;
    CameraPipeline cameraPipeline;
    public static CameraPipeline.PortalConfiguration portalConfiguration = new CameraPipeline.PortalConfiguration();
    //endregion
    boolean fieldOriented = true;
    MultipleTelemetry multipleTelemetry;
    Telemetry dashboardTelemetry;
    @UpdateAutomatically
    Toggle rotateToggle = new Toggle(Input.KeyCode.Gamepad1A); // toggle to check rotation fix
    Arm arm;

    @Override
    public void runOpMode() {


        initSystems();
        waitForStart();
        double targetHeading = drive.getHeading();
        drive.resetOrientation(0);
        while (!isStopRequested() && opModeIsActive()) {
            Input.updateControls();

            handleForceQuit();

            if (gamepad1.start) {
                if (gamepad1.x) {
                    drive.resetOrientation(0);
                    targetHeading = drive.getHeading();
                }
                continue;
            }
//            arm.setPower(gamepad2.left_stick_y);


            final double boostK = 0.5;
            double boost = gamepad1.right_trigger * boostK + (1 - boostK) * 2;
            double y = pow(gamepad1.left_stick_y) * boost;
            double x = pow(-gamepad1.left_stick_x) * boost;
            double turn = pow(gamepad1.right_stick_x * boost);
//            if (gamepad2.a) {
//                arm.goToPos(Arm.Position.One);
//            }
//            if (gamepad2.b) {
//                arm.goToPos(Arm.Position.Two);
//            }
//            if (gamepad2.y) {
//                arm.goToPos(Arm.Position.Three);
//            }

            if (gamepad1.dpad_left) drive.setPower(0, 0, 0.1);
            else if (gamepad1.dpad_right) drive.setPower(0, 0, -0.1);
            else if (gamepad1.dpad_up) drive.setPower(-0.1, 0, 0);
            else if (gamepad1.dpad_down) drive.setPower(0.1, 0, 0);
            else {
                if (cameraPipeline.getDetections().size() > 0) { // Some tag is detected
                    drive.setPowerOriented(y, x, turn, fieldOriented);
                    allowMovement = true;
                    AprilTagDetection tag = cameraPipeline.getSpecificTag(2); // Get the right wall middle tag
                    if (tag == null) {
                        continue;
                    }
                    if (rotateToggle.isClicked()) {
                        if (!drive.busy)
                            cameraPipeline.lockOnTag(CameraPipeline.AprilTags.BlueCenter, 0.3, drive);
                    }
//                    cameraPipeline.printTelemetryData(tag);
                    angle = -tag.ftcPose.yaw;
                } else {
                    drive.setPowerOriented(y, x, turn, fieldOriented);
                    allowMovement = true;
                }
                multipleTelemetry.addData("tags detected ", (int) cameraPipeline.getDetections().size());
                multipleTelemetry.addData("Allow Movement", allowMovement);
//                telemetry.addData("Timer", arm.timer.seconds());
//                telemetry.addData("State", arm.stateMachine.getCurrentState().toString());
                multipleTelemetry.update();
            }
        }
    }

    void zeroOnTarget() {
        AprilTagDetection tag = cameraPipeline.getSpecificTag(2); // Get the right wall middle tag
        drive.turn(-tag.ftcPose.yaw, 0.11);
    }

    void initSystems() {
        dashboard = FtcDashboard.getInstance();
        dashboardTelemetry = dashboard.getTelemetry();
        multipleTelemetry = new MultipleTelemetry(dashboardTelemetry, telemetry);
//        arm = new Arm(this);
//        arm.init(hardwareMap);
        telemetry.addData(">", "Init in progress...");
        telemetry.update();

        drive.init(hardwareMap);
        cameraPipeline = new CameraPipeline("cam", new Size(800, 448), hardwareMap, multipleTelemetry, portalConfiguration);
        Input.init(this, gamepad1, gamepad2);
        telemetry.addData(">", "Init finished. Press START");
        telemetry.update();
    }


    public double pow(double x) {
        return Math.pow(x, 2) * Math.signum(x);
    }

    void handleForceQuit() {
        if (Input.GetKeyPressed(Input.KeyCode.Gamepad1X) && Input.GetKeyPressed(Input.KeyCode.Gamepad1Start) && Input.GetKeyPressed(Input.KeyCode.Gamepad1Options)) { // Force stops the OpMode
            requestOpModeStop();
        }
    }

}
