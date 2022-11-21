package org.firstinspires.ftc.teamcode.power_play.Autonomous;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.RobotLog;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.freight_frenzy.util.DuckLine;
import org.firstinspires.ftc.teamcode.power_play.util.DriveClass;
import org.firstinspires.ftc.teamcode.power_play.util.Lift;
import org.firstinspires.ftc.teamcode.power_play.util.Location;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.easyopencv.OpenCvWebcam;

public class AutoFlow {
    private LinearOpMode opMode = null;
    private DriveClass drive = null;

    final double tile = 0.6;



    public enum ALLIANCE {
        BLUE(1),
        RED(-1);

        public int mul;

        ALLIANCE(int mul) {
            this.mul = mul;
        }
    }

    public enum StartPos {
        front(1),
        back(-1);

        int mul;

        StartPos(int mul) {
            this.mul = mul;
        }
    }

    public enum Auto {
        SHORT(1),
        LONG(2),
        PARK(3),
        FULL(4),
        CYCLING(5);

        public int value;

        Auto(int value) {
            this.value = value;
        }
    }

    final double robotLength = 0.4064;
    final int screenWidth = 640;
    final int screenHeight = 360;

    Location startLocation = new Location(0.9, robotLength / 2);
    Location coneLocation = new Location(1.5, 1.5,90);



    Auto auto;
    ALLIANCE alliance;
    StartPos startPos;


    public AutoFlow(LinearOpMode opMode, ALLIANCE alliance, StartPos startPos, Auto auto) {
        this.opMode = opMode;
        this.alliance = alliance;
        this.startPos = startPos;
        this.auto = auto;


        this.drive = new DriveClass(opMode, DriveClass.ROBOT.CONSTANTIN, startLocation, DriveClass.USE_ENCODERS | DriveClass.USE_DASHBOARD_FIELD, alliance == ALLIANCE.BLUE ? DriveClass.DriveMode.BLUE : DriveClass.DriveMode.RED);
    }

    public void init() {
        //initWebcam();
        //ToDo inits
        drive.init(opMode.hardwareMap);
        opMode.telemetry.update();

    }

    public void run() {

        drive.goToLocation(coneLocation,1,0.3,coneLocation.angle);

        //Autonomous starts

    }
}

