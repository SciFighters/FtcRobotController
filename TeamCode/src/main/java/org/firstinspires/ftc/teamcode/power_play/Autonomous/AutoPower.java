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

public class AutoPower {
    private LinearOpMode opMode = null;
    private DriveClass drive = null;
    private Lift lift = null;

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
        BOTTOM(1),
        TOP(-1);

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
    //region Locations
    Location startLocation = new Location(0.9, robotLength / 2);
    Location coneLocation = new Location(1.5, 1.5, 90); // also park 3
    Location highJunction = new Location(0.9, 1.5, -45); // also park 2
    Location highJunctionSafe = new Location(0.3, 1.2);
    Location medJunction = new Location(0.9, 1.5, -135); // also park 2
    Location park1 = new Location(0.3, 1.5);
    Location park2 = highJunction;
    Location park3 = coneLocation;
    //endregion


    Auto auto;
    ALLIANCE alliance;
    StartPos startPos;


    public AutoPower(LinearOpMode opMode, ALLIANCE alliance, StartPos startPos, Auto auto) {
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
//Autonomous starts
        if (auto == auto.FULL) {
            lift.gotoLevel(Lift.LiftLevel.Third);
            for (int i = 0; i < 4; i++) {
                drive.goToLocation(highJunction, 1, 0.2, highJunction.angle);
                lift.grabber(true);
                lift.gotoLevel(Lift.LiftLevel.Floor);

                drive.goToLocation(coneLocation, 1, 0.2, coneLocation.angle);
                lift.grabber(false);
                lift.gotoLevel(Lift.LiftLevel.Third);
            }


        }
        if (auto==auto.LONG){ //backup, less points
            lift.gotoLevel(Lift.LiftLevel.Third);
            drive.goToLocation(highJunctionSafe,1,0.2,highJunctionSafe.angle);
            lift.grabber(true);
            for (int i = 0; i < 3; i++) {
                lift.gotoLevel(Lift.LiftLevel.Floor);
                drive.goToLocation(coneLocation,1,0.2,coneLocation.angle);
                lift.grabber(false);

                lift.gotoLevel(Lift.LiftLevel.Second);
                drive.goToLocation(medJunction,1,0.2,medJunction.angle);
                lift.grabber(false);

            }
        }
    }

}


