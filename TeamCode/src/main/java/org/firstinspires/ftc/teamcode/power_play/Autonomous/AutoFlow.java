package org.firstinspires.ftc.teamcode.power_play.Autonomous;

import android.util.Log;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.freight_frenzy.util.DuckLine;
import org.firstinspires.ftc.teamcode.power_play.util.CameraInitializer;
import org.firstinspires.ftc.teamcode.power_play.util.DriveClass;
import org.firstinspires.ftc.teamcode.power_play.util.Lift;
import org.firstinspires.ftc.teamcode.power_play.util.Location;
import org.firstinspires.ftc.teamcode.power_play.util.SleevePipeline;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.easyopencv.OpenCvWebcam;

public class AutoFlow {
    private LinearOpMode opMode = null;
    private DriveClass drive = null;
    private SleevePipeline pipeline;
    Location coneLocation = new Location(-0.50, 1.55, 90); // also park 3
    Location highJunction = new Location(-0.72, 1.62, -135); // also park 2
    Location highJunctionSafe = new Location(0.3, 1.2);
    Location conePushLocation = new Location(-0.9, 1.7);
    Location pre_highJunction = new Location(-0.9, 1.5);

    Location medJunction = new Location(0.9, 1.5, -135); // also park 2
    private ParkingPosition parkingPosition = ParkingPosition.two;
    final double tile = 0.6;

    //region Enums


    public enum SIDE {
        LEFT(-1), RIGHT(1);

        public int mul;

        SIDE(int mul) {
            this.mul = mul;
        }
    }

    public enum ParkingPosition {
        one(new Location(-1.5, 1.45, 180)),
        two(new Location(-0.9, 1.45, 180)),
        three(new Location(-0.3, 1.45, 180));

        public Location location;

        ParkingPosition(Location location) {
            this.location = location;
        }
    }

    public enum StartPos {
        LEFT(1),
        RIGHT(-1);

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

    final double robotLength = 0.4404;
    final int screenWidth = 640;
    final int screenHeight = 360;


    Location startLocation = new Location(-0.9, robotLength / 2,180); // LEFT
    Auto auto;
    StartPos startPos;
    private Lift lift = null;

    public AutoFlow(LinearOpMode opMode, StartPos startPos, Auto auto) {
        this.opMode = opMode;
        this.startPos = startPos;
        this.auto = auto;

        this.drive = new DriveClass(opMode, DriveClass.ROBOT.CONSTANTIN, startLocation, DriveClass.USE_ENCODERS | DriveClass.USE_DASHBOARD_FIELD, startPos == StartPos.LEFT ? DriveClass.DriveMode.LEFT : DriveClass.DriveMode.LEFT);
//        if(auto == Auto.PARK) startLocation = startLocation.offsetX()

    }


    public void init() {

        CameraInitializer.initialize(opMode, "webcam", 320, 180, pipeline, false);
        drive.init(opMode.hardwareMap);
        this.lift = new Lift();
        lift.init(opMode.hardwareMap);
        lift.grabber(true); // closes grabbers, init
        pipeline = new SleevePipeline();
        opMode.sleep(1000);
        opMode.telemetry.addData("camara is on", pipeline.getParkingLocation());
        SleevePipeline.ParkingLocation loc = pipeline.getParkingLocation();
        if (loc == SleevePipeline.ParkingLocation.One) {
            opMode.telemetry.addLine("1");
        } else if (loc == SleevePipeline.ParkingLocation.Two) {
            opMode.telemetry.addLine("2");
        } else if(loc == SleevePipeline.ParkingLocation.Three) {
            opMode.telemetry.addLine("3");
        }
        opMode.telemetry.addData("heading", drive.getHeading());
        opMode.telemetry.update();
    }

    public void gotoParkingPosition(ParkingPosition parkingPosition) {
        opMode.telemetry.addData("Going to", "parking position: x=" + parkingPosition.location.x + " y=" + parkingPosition.location.y);
        opMode.telemetry.update();
        drive.goToLocation(new Location(drive.getPosX(), parkingPosition.location.y, 180), 1, 0.05, 0);
        drive.goToLocation(parkingPosition.location, 1, 0.05, 0);
    }

    private void parkAtConeLocation() {

    }

    public void run() {
        opMode.telemetry.addData("camara is on", pipeline.getParkingLocation());
        SleevePipeline.ParkingLocation loc = pipeline.getParkingLocation();
        if (loc == SleevePipeline.ParkingLocation.One) {
            opMode.telemetry.addLine("1");

        } else if (loc == SleevePipeline.ParkingLocation.Two) {
            opMode.telemetry.addLine("2");
        } else if(loc == SleevePipeline.ParkingLocation.Three) {
            opMode.telemetry.addLine("3");
        }

        if (auto == Auto.FULL) {
            lift.gotoLevel(Lift.LiftLevel.Third, true, null);
            for (int i = 0; i < 4; i++) { // Going to put 4 cones
                drive.goToLocation(highJunction, 1, 0.06, 0);
                opMode.sleep((int) (2 * 1000));
                lift.grabber(false); // opens grabber
                opMode.sleep((int) (2 * 1000));
                lift.gotoLevel(Lift.LiftLevel.Floor, true, null); // TODO: check if the height right? (cone pile)
                opMode.sleep((int) (2 * 1000));
                drive.goToLocation(coneLocation, 1, 0.2, 0); // TODO: change cone location (1.5, 1.5 ? )
                opMode.sleep((int) (2 * 1000));
                lift.grabber(true); //(Changed to true) TODO: check validity of grabber ability
                opMode.sleep((int) (2 * 1000));
                lift.gotoLevel(Lift.LiftLevel.Third, true, null);
                opMode.sleep((int) (2 * 1000));
            }
        }
        //TODO: DO NOT DELETE CODE
        if (auto == auto.LONG) { //backup, less points
            lift.gotoLevel(Lift.LiftLevel.Third, true, null);
            drive.goToLocation(highJunctionSafe, 1, 0.2, highJunctionSafe.angle);

            lift.grabber(false); //(Changed to false) TODO: check
            for (int i = 0; i < 3; i++) {
                lift.gotoLevel(Lift.LiftLevel.Floor, true, null);
                drive.goToLocation(coneLocation, 1, 0.2, coneLocation.angle);
                lift.grabber(true); //(Changed to true)
                final int tickDiff = 30;
                lift.gotoLevel(Lift.LiftLevel.Second, -(i * tickDiff), true, null);

                drive.goToLocation(medJunction, 1, 0.2, medJunction.angle);
                lift.grabber(false);

            }
        }

        // TODO: Parking in the right place... (OpenCV)
        if (auto._isParking) gotoParkingPosition(parkingPosition);
        if (auto == Auto.PARK) drive.turn(-179, 0.5);
        //
    }

    public void placeFirstCone() {
        lift.gotoLevel(Lift.LiftLevel.Third, true, null);
        //drive.goTo(drive.getPosX(), 1,0.5, -100, 0.15, 3); // goes to high junction position
//        drive.goTo(drive.getPosX(), 1.4,0.5, -100, 0.15, 3); // goes to high junction position
//        opMode.sleep(500);

        drive.goTo(drive.getPosX(), 1.3, 0.5, drive.getHeading(), 0.07, 0);
        drive.goTo(drive.getPosX(), 1.6, 0.5, 135, 0.07, 0);
        drive.goTo(drive.getPosX(), 1.3, 0.3, 180, 0.07, 0);

        drive.goToLocation(highJunction, 0.4, 0.01, 3);
        opMode.sleep(500);
        lift.grabber(false);
    }

    public void placeCones() {

        lift.gotoLevel(Lift.LiftLevel.cone5, false, null); // goes to the highest cone
        lift.grabber(false); // opens grabber
        opMode.sleep(50);
        drive.goToLocation(new Location(0, coneLocation.y, drive.getHeading()),
                0.5, 0.1, 3); // goes to robot x, and cone y
        drive.turnTo(-90, 0.5);
        drive.goToLocation(new Location(-0.2, coneLocation.y, drive.getHeading()),
                0.5, 0.1, 3); // goes to robot x, and cone y
        drive.turnTo(-90, 0.5);
        drive.goToLocation(new Location(-0.50, drive.getPosY(), -90), 0.5, 0.1, 3); // goes to cone x, and robot y
//        drive.goToLocation(new Location(drive.getPosX() + 0.4, drive.getPosY(), -90), 0.5, 0.1, 3);
        opMode.sleep(300);
        lift.grabber(true); // closes grabber
        opMode.sleep(500);
        lift.gotoLevel(Lift.LiftLevel.Third, false, null);
        opMode.sleep(500);
        lift.toggleFlip(null);
        drive.goToLocation(new Location(drive.getPosX() + 0.5,
                        drive.getPosY(),
                        drive.getHeading()),
                0.5, 0.1, 3);
        drive.turnTo(-180, 0.5);
        opMode.sleep(500);
        drive.goToLocation(new Location(highJunction.x + 0.1,
                        highJunction.y,
                        drive.getHeading()),
                0.3, 0.1, 3);
        opMode.sleep(400);
        lift.grabber(false);
        opMode.sleep(300);
        gotoParkingPosition(ParkingPosition.three);
//        lift.gotoLevel(Lift.LiftLevel.Third, false); // goes to max height of elevator
//        drive.turn(90, 0.5);
//        drive.goToLocation(new Location(highJunctionSafe.x, highJunctionSafe.y, 90), 0.5, 0.1, 3); // goes to high junction position
//        lift.toggleFlip(); // changes side of flip motors
//        lift.grabber(false); // opens grabber

    }

    public void run2() {
        gotoParkingPosition(parkingPosition);
    }
    public void test(){
        SleevePipeline.ParkingLocation loc = pipeline.getParkingLocation();
        Log.e("Sci", "camera initialization failed: " + loc);
        opMode.telemetry.addData("camara is on", loc);

        if (loc == SleevePipeline.ParkingLocation.One) {
            opMode.telemetry.addLine("1");
            parkingPosition = ParkingPosition.one;
        } else if (loc == SleevePipeline.ParkingLocation.Two) {
            opMode.telemetry.addLine("2");
            parkingPosition = ParkingPosition.two;
        } else if(loc == SleevePipeline.ParkingLocation.Three) {
            opMode.telemetry.addLine("3");
            parkingPosition = ParkingPosition.three;
        } else {
            opMode.telemetry.addLine("NOTHING");
            opMode.telemetry.update();
            parkingPosition = ParkingPosition.two;
        }
        placeFirstCone();
        gotoParkingPosition(parkingPosition);
    }
}

