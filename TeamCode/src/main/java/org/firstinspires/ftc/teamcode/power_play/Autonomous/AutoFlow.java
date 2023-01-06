package org.firstinspires.ftc.teamcode.power_play.Autonomous;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.power_play.util.DriveClass;
import org.firstinspires.ftc.teamcode.power_play.util.Lift;
import org.firstinspires.ftc.teamcode.power_play.util.Location;

public class AutoFlow {
    private LinearOpMode opMode = null;
    private DriveClass drive = null;
    Location coneLocation = new Location(-0.85, 1.5, 90); // also park 3
    Location highJunction = new Location(1.8, 1.5, -45); // also park 2
    Location highJunctionSafe = new Location(0.3, 1.2);
    Location medJunction = new Location(0.9, 1.5, -135); // also park 2
    final double tile = 0.6;

    //region Enums
    public enum ALLIANCE {
        BLUE(1), RED(-1);

        public int mul;

        ALLIANCE(int mul) {
            this.mul = mul;
        }
    }

    public enum SIDE {
        LEFT(-1), RIGHT(1);

        public int mul;

        SIDE(int mul) {
            this.mul = mul;
        }
    }

    public enum ParkingPosition {
        one(new Location(-0.75, 0.93)), two(new Location(0, 0.80)), three(new Location(0.70, 0.93));

        public Location location;

        ParkingPosition(Location location) {
            this.location = location;
        }
    }

    public enum StartPos {
        front(1), back(-1);

        int mul;

        StartPos(int mul) {
            this.mul = mul;
        }
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
    //endregion

    final double robotLength = 0.4064;
    final int screenWidth = 640;
    final int screenHeight = 360;


    Location startLocation = new Location(0, robotLength / 2);


    Auto auto;
    ALLIANCE alliance;
    StartPos startPos;
    private Lift lift = null;

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
        this.lift = new Lift();
        lift.init(opMode.hardwareMap);
        lift.grabber(true); // closes grabbers, init

    }


    public void gotoParkingPosition(ParkingPosition parkingPosition) {
        double parkingPosition_power = 0.8;
        double parkingPosition_tolerance = 0.1;
        double parkingPosition_timeout = 0;
        drive.goToLocation(new Location(drive.getPosX(), parkingPosition.location.y), parkingPosition_power, parkingPosition_tolerance, parkingPosition_timeout);
        drive.goToLocation(parkingPosition.location, parkingPosition_power, parkingPosition_tolerance, parkingPosition_tolerance);
    }

    private void parkAtConeLocation() {

    }

    public void run() {
        if (auto == Auto.FULL) {
            lift.gotoLevel(Lift.LiftLevel.Third, true);
            for (int i = 0; i < 4; i++) { // Going to put 4 cones
                drive.goToLocation(highJunction, 1, 0.06, 0);
                opMode.sleep((int) (2 * 1000));
                lift.grabber(false); // opens grabber
                opMode.sleep((int) (2 * 1000));
                lift.gotoLevel(Lift.LiftLevel.Floor, true); // TODO: check if the height right? (cone pile)
                opMode.sleep((int) (2 * 1000));
                drive.goToLocation(coneLocation, 1, 0.2, 0); // TODO: change cone location (1.5, 1.5 ? )
                opMode.sleep((int) (2 * 1000));
                lift.grabber(true); //(Changed to true) TODO: check validity of grabber ability
                opMode.sleep((int) (2 * 1000));
                lift.gotoLevel(Lift.LiftLevel.Third, true);
                opMode.sleep((int) (2 * 1000));
            }
        }
        //TODO: DO NOT DELETE CODE
        if (auto == auto.LONG) { //backup, less points
            lift.gotoLevel(Lift.LiftLevel.Third, true);
            drive.goToLocation(highJunctionSafe, 1, 0.2, highJunctionSafe.angle);

            lift.grabber(false); //(Changed to false) TODO: check
            for (int i = 0; i < 3; i++) {
                lift.gotoLevel(Lift.LiftLevel.Floor, true);
                drive.goToLocation(coneLocation, 1, 0.2, coneLocation.angle);
                lift.grabber(true); //(Changed to true)
                final int tickDiff = 30;
                lift.gotoLevel(Lift.LiftLevel.Second, -(i * tickDiff), true);

                drive.goToLocation(medJunction, 1, 0.2, medJunction.angle);
                lift.grabber(false);

            }
        }

        // TODO: Parking in the right place... (OpenCV)
        if (auto._isParking) parkAtConeLocation();

        //
    }

    public void placeFirstCone() {
        lift.gotoLevel(Lift.LiftLevel.ThirdFront, false);
        drive.goToLocation(new Location(0, 1.5), 0.5, 0.1, 3); // goes to high junction position
        drive.goToLocation(new Location(0, 1.6), 0.4, 0.1, 3);
        drive.turnTo(50, 0.5);
        drive.goToLocation(new Location(
                drive.getPosX() + 0.14,
                drive.getPosY() + 0.14,
                drive.getHeading()),
                0.4, 0.1, 3);
        drive.goToLocation(new Location(0, 1.6), 0.4, 0.1, 3);
        lift.grabber(false);

    }
    public void placeCones() {
        lift.gotoLevel(Lift.LiftLevel.cone5, false); // goes to the highest cone
        lift.grabber(false); // opens grabber
        drive.goToLocation(new Location(drive.getPosX(), coneLocation.y), 0.5, 0.1, 3); // goes to robot x, and cone y
        drive.turn(-90, 0.5);
        drive.goToLocation(new Location(coneLocation.x, drive.getPosY(), -90), 0.5, 0.1, 3); // goes to cone x, and robot y
//        drive.goToLocation(new Location(drive.getPosX() + 0.4, drive.getPosY(), -90), 0.5, 0.1, 3);
        opMode.sleep(500);
        lift.grabber(true); // closes grabber
        opMode.sleep(500);
//        lift.gotoLevel(Lift.LiftLevel.Third, false); // goes to max height of elevator
//        drive.turn(90, 0.5);
//        drive.goToLocation(new Location(highJunctionSafe.x, highJunctionSafe.y, 90), 0.5, 0.1, 3); // goes to high junction position
//        lift.toggleFlip(); // changes side of flip motors
//        lift.grabber(false); // opens grabber

    }
}

