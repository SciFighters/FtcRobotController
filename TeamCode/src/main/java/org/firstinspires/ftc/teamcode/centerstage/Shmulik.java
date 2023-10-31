package org.firstinspires.ftc.teamcode.centerstage;

import android.util.Size;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Gamepad;

import org.firstinspires.ftc.teamcode.centerstage.ArmPIDTesting.Arm;
import org.firstinspires.ftc.teamcode.centerstage.Systems.CameraPipeline;
import org.firstinspires.ftc.teamcode.centerstage.Systems.DriveClass;
import org.firstinspires.ftc.teamcode.centerstage.util.Input.Input;
import org.firstinspires.ftc.teamcode.centerstage.util.Input.Toggle;
import org.firstinspires.ftc.teamcode.centerstage.util.Input.UpdateAutomatically;
import org.firstinspires.ftc.teamcode.power_play.util.Location;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;

@TeleOp(group = "CENTERSTAGE")
public class Shmulik extends LinearOpMode {
    DriveClass drive = new DriveClass(this,
            DriveClass.ROBOT.JACCOUSE,
            new Location(-0.9, 0.4404 / 2, 180),
            DriveClass.USE_ENCODERS | DriveClass.USE_BRAKE,
            DriveClass.DriveMode.LEFT);
    boolean allowMovement = true; // Flag to control movement
    @UpdateAutomatically
    Toggle rotateToggle = new Toggle(Input.KeyCode.Gamepad1A); // toggle to check rotation fix
    double angle = 0;
    final double maxTagSize = 239;
    CameraPipeline cameraPipeline;
    Gamepad.RumbleEffect rumbleEffect;
    boolean fieldOriented = true;
    Arm arm;

    @Override
    public void runOpMode() {
        rumbleEffect = new Gamepad.RumbleEffect.Builder()
                .addStep(0.0, 1.0, 500)
                .addStep(0.0, 0.0, 300)
                .addStep(1.0, 0.0, 250)
                .build();
        initSystems();
        waitForStart();
        double targetHeading = drive.getHeading();
        drive.resetOrientation(0);
        while (!isStopRequested() && opModeIsActive()) {
            Input.updateControls(gamepad1, gamepad2);
            telemetry.addData("Rotate Toggle", rotateToggle.getState() + " " + rotateToggle.getMapping());
            if (gamepad1.start) {
                gamepad1.runRumbleEffect(rumbleEffect);
                if (gamepad1.x) {
                    drive.resetOrientation(0);
                    targetHeading = drive.getHeading();
                }
                continue;
            }
            arm.setArmPower(0, gamepad2.right_stick_y);
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
                    drive.setPowerOriented(y, x, turn, fieldOriented);
                    allowMovement = true;
                    AprilTagDetection tag = cameraPipeline.getSpecificTag(5); // Get the right wall middle tag
                    if (tag == null) {
                        continue;
                    }
                    if (rotateToggle.isPressed()) {
                        if (!drive.busy)
                            zeroOnTarget();
                    }
                    cameraPipeline.printTelemetryData(tag);
                    angle = -tag.ftcPose.yaw;
                } else {
                    drive.setPowerOriented(y, x, turn, fieldOriented);
                    allowMovement = true;
                }
                telemetry.addData("tags detected ", (int) cameraPipeline.getDetections().size());
                telemetry.addData("Allow Movement", allowMovement);
                telemetry.update();
            }
        }
    }

    void zeroOnTarget() {
        AprilTagDetection tag = cameraPipeline.getSpecificTag(5); // Get the right wall middle tag
        drive.turn(-tag.ftcPose.yaw, 0.11);
        tag = cameraPipeline.getSpecificTag(5); // Get the right wall middle tag
        drive.turn(-tag.ftcPose.yaw, 0.11);
    }

    void initSystems() {
        arm = new Arm(this);
        arm.init(hardwareMap);
        telemetry.addData(">", "Init in progress...");
        telemetry.update();
        drive.init(hardwareMap);
        cameraPipeline = new CameraPipeline("cam", new Size(640, 480), hardwareMap, this.telemetry);
        Input.init(this);
        telemetry.addData(">", "Init finished. Press START");
    }

    public double pow(double x) {
        return Math.pow(x, 2) * Math.signum(x);
    }


}
