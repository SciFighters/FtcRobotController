package org.firstinspires.ftc.teamcode.power_play;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.power_play.util.DriveClass;
import org.firstinspires.ftc.teamcode.power_play.util.Lift;
import org.firstinspires.ftc.teamcode.power_play.util.Location;

@TeleOp
public class RobotReveal extends LinearOpMode {
    Lift lift = new Lift();
    DriveClass drive = new DriveClass(this, DriveClass.ROBOT.JACCOUSE, new Location(0, 0), DriveClass.USE_ENCODERS | DriveClass.USE_BRAKE, DriveClass.DriveMode.LEFT);
    double batteryLevel;
    double batteryPercentage = (batteryLevel - 11.7) / (12.6 - 11.7);

    @Override
    public void runOpMode() {
        this.telemetry = new MultipleTelemetry(this.telemetry, FtcDashboard.getInstance().getTelemetry());
        telemetry.addLine("Starting Initializing");
        telemetry.update();

        drive.init(hardwareMap);
        lift.init(hardwareMap);
        telemetry.addLine("Finished Initializing");
        telemetry.update();

        waitForStart();

        int turningCount = 0;
        double targetHeading = 0;

        drive.resetOrientation(0);
        while (opModeIsActive()) {
            if (gamepad1.left_stick_y < 0) {
                drive.setPower(0.5, 0, 0);
            }
            if (gamepad1.left_stick_y > 0) {
                drive.setPower(-0.5, 0, 0);
            }
            if (gamepad1.a) {
                drive.setPower(0, 0, 0);
            }
        }
    }

    public double pow(double x) {
        return Math.pow(x, 2) * Math.signum(x);
    }
}
