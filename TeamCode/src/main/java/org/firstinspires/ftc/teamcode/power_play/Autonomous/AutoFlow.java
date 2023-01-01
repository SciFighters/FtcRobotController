package org.firstinspires.ftc.teamcode.power_play.Autonomous;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.power_play.util.DriveClass;
import org.firstinspires.ftc.teamcode.power_play.util.Location;

public class AutoFlow {
    private LinearOpMode opMode = null;
    private DriveClass drive = null;

    final double tile = 0.6;

    //region Enums
    public enum ALLIANCE {
        BLUE(1),
        RED(-1);

        public int mul;

        ALLIANCE(int mul) {
            this.mul = mul;
        }
    }

    public enum ParkingPosition {
        one(new Location(-0.80, 0.80)),
        two(new Location(0, 0.80)),
        three(new Location(0.70, 0.80));

        public Location location;

        ParkingPosition(Location location) {
            this.location = location;
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
    //endregion

    final double robotLength = 0.4064;
    final int screenWidth = 640;
    final int screenHeight = 360;


    //Location startLocation = new Location(0.9, robotLength / 2);
    Location startLocation = new Location(0, robotLength / 2);
    Location coneLocation = new Location(1.5, 1.5, 90);


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

    public void gotoParkingPosition(int positionIndex) {
        double power = 0.8;
        double tolerance = 1;
        double timeout = 0;
        switch (positionIndex) {
            case 1:
                drive.goToLocation(ParkingPosition.one.location, power, tolerance, timeout);
                break;
            case 2:
                drive.goToLocation(ParkingPosition.two.location, power, tolerance, timeout);
                break;
            case 3:
                drive.goToLocation(ParkingPosition.three.location, power, tolerance, timeout);
                break;
        }
    }

    public void run() {

        //drive.goToLocation(coneLocation, 1, 0.3, coneLocation.angle);
        drive.goToLocation(new Location(0,0.80), 1, 0.3, coneLocation.angle);
        drive.goToLocation(new Location(0.70,0.80), 1, 0.3, coneLocation.angle);

        //Autonomous starts

    }
}

