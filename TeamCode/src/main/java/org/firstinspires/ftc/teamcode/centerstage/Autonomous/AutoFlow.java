package org.firstinspires.ftc.teamcode.centerstage.Autonomous;

import android.util.Size;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.centerstage.Systems.CameraPipeline;
import org.firstinspires.ftc.teamcode.centerstage.Systems.DriveClass;
import org.firstinspires.ftc.teamcode.freight_frenzy.util.MathUtil;
import org.firstinspires.ftc.teamcode.power_play.util.Location;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;

public class AutoFlow {
    private DriveClass drive;
    LinearOpMode opMode = null;
    final double robotLength = 0.4404;
    final double tile = 0.6;
    Location startLocation = new Location(0.9, robotLength / 2, 0); // PIXEL_STACK
    CameraPipeline cameraPipeline;
    Alliance alliance;

    public enum StartPos {
        PIXEL_STACK(1),
        BACKSTAGE(-1);

        int mul;

        StartPos(int mul) {
            this.mul = mul;
        }
    }

    public enum Alliance {
        BLUE, RED
    }

    public enum Auto {
        PARK(1, true), SHORT(2, true), LONG(3, true), FULL(4, true), CYCLING(5, true);

        public int value;
        public boolean _isParking;

        Auto(int value, boolean isParking) {
            this.value = value;
            this._isParking = isParking;
        }
    }

    public AutoFlow(LinearOpMode opMode, Alliance alliance, StartPos startPos, Auto auto) {
        this.alliance = alliance;
        this.opMode = opMode;
        this.drive = new DriveClass
                (
                        opMode, DriveClass.ROBOT.CONSTANTIN,
                        startLocation,
                        DriveClass.USE_ENCODERS | DriveClass.USE_BRAKE | DriveClass.USE_DASHBOARD_FIELD,
                        DriveClass.DriveMode.LEFT
                );
    }

    public void init() {
        cameraPipeline = new CameraPipeline("cam", new Size(800, 448), opMode.hardwareMap, opMode.telemetry);
        drive.init(opMode.hardwareMap);
    }

    public void test() {
        drive.goToLocation(new Location(drive.getPosX(), tile * 3 + 0.4), 1, 0, 0.15, 0);
        drive.goToLocation(new Location(drive.getPosX() - tile * 3, drive.getPosY()), 1, -90, 0.15, 0);
        drive.goToLocation(new Location(drive.getPosX() - tile * 2, tile * 2), 1, -90, 0.05, 0);
        lockOnTag(CameraPipeline.AprilTags.BlueLeft, 0.3, 3);
    }

    /**
     * Moves the robot to a tag in the y axis
     **/
    public void lockOnTag(int tagID, double power, double timeout) {
        double initialX = drive.getPosX(), initialY = drive.getPosY(), initialHeading = drive.getHeading();
        double distance = 0;
        ElapsedTime time = new ElapsedTime(); // A timer to have a timeout
        time.reset();
        do {
            AprilTagDetection tag = cameraPipeline.getSpecificTag(tagID);
            distance = tag.ftcPose.x;

            if (!MathUtil.approximately(distance, 0.1, 0.1)) {
                // Adjust robot position based on the distance
                drive.goToLocation(
                        new Location(
                                initialX, drive.getPosY() + distance
                        ), 1, drive.getHeading(), 0.05, 0);

                // Update tag and distance after movement
                tag = cameraPipeline.getSpecificTag(tagID);
                distance = tag.ftcPose.x;
            }
        } while (MathUtil.approximately(distance, 0.1, 0.1) && time.seconds() < timeout);
    }

    public void lockOnTag(CameraPipeline.AprilTags tag, double power, double timeout) {
        lockOnTag(tag.ID, power, timeout);
    }
}
