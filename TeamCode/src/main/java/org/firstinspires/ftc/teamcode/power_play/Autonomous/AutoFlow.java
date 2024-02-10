package org.firstinspires.ftc.teamcode.power_play.Autonomous;

import android.util.Log;

import androidx.annotation.Nullable;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.power_play.util.CameraInitializer;
import org.firstinspires.ftc.teamcode.power_play.util.DriveClass;
import org.firstinspires.ftc.teamcode.power_play.util.Lift;
import org.firstinspires.ftc.teamcode.power_play.util.Location;
import org.firstinspires.ftc.teamcode.power_play.util.SleevePipeline;
import org.openftc.easyopencv.OpenCvWebcam;

import java.util.Set;

public class AutoFlow {
    private OpenCvWebcam camRod = null;
    private OpenCvWebcam camTower = null;

    private SleevePipeline pipelineSleeve = new SleevePipeline();

    final double tile = 0.6;
    final double robotLength = 0.4404;
    final int screenWidth = 640;
    final int screenHeight = 360;
    //region positions
    Location highJunction0 = new Location(0, 1.80, 180); // also park 2
    Location highJunction1 = new Location(0, 1.75, 180); // also park 2

    Location prehighJunction1 = new Location(-0.75, 1.7, -112);

    Location preStack = new Location(-1.2, 1.58, -90);
    Location prePark1 = new Location(-0.9, 1.58, -90);
    Location coneLocation1 = new Location(-1.5, 1.58, -90);
    Location coneLocation = new Location(-1.5, 1.53, -90); // also park 3
    Location highJunction = new Location(-0.589, 1.66, 180); // also park 2
    Location midJunction = new Location(-0.583, 1.38, 0); // also park 2
    Location highJunctionSafe = new Location(0.3, 1.2);
    Location conePushLocation = new Location(-0.9, 1.7);
    Location pre_highJunction = new Location(-0.9, 1.5);
    Location startLocation = new Location(-0.9, robotLength / 2, 180); // PIXEL_STACK
    Auto auto;

    public enum StartPos {
        LEFT(1),
        RIGHT(-1);

        int mul;

        StartPos(int mul) {
            this.mul = mul;
        }
    }

    //region Enums
    StartPos startPos;

    private LinearOpMode opMode = null;
    private DriveClass drive = null;
    //endregion
    //endregion
    private ParkingPosition parkingPosition;
    public Lift lift = null;

    public AutoFlow(LinearOpMode opMode, StartPos startPos, Auto auto) {
        this.opMode = opMode;
        this.startPos = startPos;
        this.auto = auto;
        if (startPos == startPos.RIGHT) {
            highJunction1.flipX();
            highJunction1.flipAngle();
            prePark1.flipX();
            prePark1.flipAngle();
            coneLocation1.flipX();
            coneLocation1.flipAngle();
            startLocation.flipX();
            preStack.flipX();
            preStack.flipAngle();
        }

        this.drive = new DriveClass(opMode, DriveClass.ROBOT.CONSTANTIN, startLocation, DriveClass.USE_ENCODERS | DriveClass.USE_BRAKE | DriveClass.USE_DASHBOARD_FIELD, startPos == StartPos.LEFT ? DriveClass.DriveMode.LEFT : DriveClass.DriveMode.LEFT);
        this.lift = new Lift();

//        if(auto == Auto.PARK) startLocation = startLocation.offsetX()
    }

    public void init() {
        camRod = CameraInitializer.initialize(opMode, "webcam_rod", screenWidth, screenHeight, pipelineSleeve, true);
//        camTower = CameraInitializer.initialize(opMode, "webcam_tower", screenWidth, screenHeight, null, false);

        drive.init(opMode.hardwareMap);
        lift.init(opMode.hardwareMap, opMode);
        lift.grabber(true); // closes grabbers, init

//        opMode.telemetry.addData("camara is on", pipeline.getParkingLocation());
//        SleevePipeline.ParkingLocation loc = pipeline.getParkingLocation();
//        if (loc == SleevePipeline.ParkingLocation.One) {
//            opMode.telemetry.addLine("1");
//        } else if (loc == SleevePipeline.ParkingLocation.Two) {
//            opMode.telemetry.addLine("2");
//        } else if(loc == SleevePipeline.ParkingLocation.Three) {
//            opMode.telemetry.addLine("3");
//        }
        SleevePipeline.ParkingLocation loc = pipelineSleeve.getParkingLocation();
        if (loc == SleevePipeline.ParkingLocation.One) this.parkingPosition = ParkingPosition.one;
        else if (loc == SleevePipeline.ParkingLocation.Two)
            this.parkingPosition = ParkingPosition.two;

        else if (loc != SleevePipeline.ParkingLocation.None)
            this.parkingPosition = ParkingPosition.three;
        else
            this.parkingPosition = ParkingPosition.two;

        if (loc == SleevePipeline.ParkingLocation.One) opMode.telemetry.addLine("1");
        else if (loc == SleevePipeline.ParkingLocation.Two) opMode.telemetry.addLine("2");
        else if (loc == SleevePipeline.ParkingLocation.Three) opMode.telemetry.addLine("3");
        opMode.telemetry.update();
    }

    public void gotoParkingPosition(ParkingPosition parkingPosition) {
        opMode.telemetry.addData("Going to", "parking position: x=" + parkingPosition.location.x + " y=" + parkingPosition.location.y);
        opMode.telemetry.update();
        drive.goToLocation(new Location(drive.getPosX(), parkingPosition.location.y, 180), 1, 0.05, 0);
        drive.goToLocation(parkingPosition.location, 1, 0.05, 0);
    }

    public void initiateConeCycling() {
        drive.goToLocationOnAxis(ParkingPosition.two.location, 0.6, 0.06, 0, DriveClass.Axis.x);
        drive.goToLocationOnAxis(ParkingPosition.two.location, 0.6, 0.06, 0, DriveClass.Axis.y);
        opMode.sleep(100);
        drive.turnTo(-90, 0.5);
    }

    public void placeCone(Location junction) {
        //drive.turnTo(-90, 0.5);
        lift.gotoDescentLevel(null);
        lift.grabber(false);
        drive.goToLocation(new Location(drive.getPosX(), coneLocation.y, coneLocation.angle), 0.5, -90, 0.01, 0);
        drive.goToLocation(new Location(coneLocation.x, coneLocation.y, drive.getHeading()), 0.5, drive.getHeading(), 0.01, 0);
        opMode.sleep(400);
        lift.grabber(true);
        opMode.sleep(400);
        lift.gotoLevel(Lift.LiftLevel.ThirdAUTO, true, null);
        opMode.sleep(300);
        //        drive.goToLocation(new Location(-1, drive.getPosY(), drive.getHeading()),
//                0.5, 0.01, 0);
//        opMode.sleep(300);
//        drive.goToLocation(new Location(drive.getPosX(), coneLocation.y - 0.05,
//                        drive.getHeading()), 0.5, 0.01, 0);
//        drive.goToLocation(coneLocation, 0.5, 0.01, 0);
        drive.goToLocation(new Location(ParkingPosition.two.location.x, ParkingPosition.two.location.y,
                drive.getHeading()), 0.5, 0.01, 0);
        drive.turnTo(180, 0.3);
        drive.goToLocation(junction, 0.5, 0.01, 0);

        opMode.sleep(500);
        lift.gotoLevel(Lift.LiftLevel.Second, false, null);
        lift.grabber(false);

    }

    public void run() {
        SleevePipeline.ParkingLocation loc = pipelineSleeve.getParkingLocation();
        opMode.telemetry.addData("camara is on", loc);
        if (loc == SleevePipeline.ParkingLocation.One) this.parkingPosition = ParkingPosition.one;
        else if (loc == SleevePipeline.ParkingLocation.Two)
            this.parkingPosition = ParkingPosition.two;
        else this.parkingPosition = ParkingPosition.three;
        opMode.telemetry.update();

        if (loc == SleevePipeline.ParkingLocation.One) opMode.telemetry.addLine("1");
        else if (loc == SleevePipeline.ParkingLocation.Two) opMode.telemetry.addLine("2");
        else if (loc == SleevePipeline.ParkingLocation.Three) opMode.telemetry.addLine("3");
        opMode.telemetry.update();
        if (auto == Auto.FULL) {
            lift.gotoLevel(Lift.LiftLevel.Third, true, null, false);
            for (int i = 0; opMode.opModeIsActive() && i < 4; i++) { // Going to put 4 cones
                drive.goToLocation(highJunction, 1, 0.06, 0);
                opMode.sleep((int) (2 * 1000));
                lift.grabber(false); // opens grabber
                opMode.sleep((int) (2 * 1000));
                lift.gotoLevel(Lift.LiftLevel.Floor, true, null, false); // TODO: check if the height right? (cone pile)
                opMode.sleep((int) (2 * 1000));
                drive.goToLocation(coneLocation, 1, 0.2, 0); // TODO: change cone locationDelta (1.5, 1.5 ? )
                opMode.sleep((int) (2 * 1000));
                lift.grabber(true); //(Changed to true) TODO: check validity of grabber ability
                opMode.sleep((int) (2 * 1000));
                lift.gotoLevel(Lift.LiftLevel.Third, true, null, false);
                opMode.sleep((int) (2 * 1000));
            }
        }
        //TODO: DO NOT DELETE CODE
        if (auto == auto.LONG) { //backup, less points
            lift.gotoLevel(Lift.LiftLevel.Third, true, null, false);
            drive.goToLocation(highJunctionSafe, 1, 0.2, highJunctionSafe.angle);

            lift.grabber(false); //(Changed to false) TODO: check
            for (int i = 0; opMode.opModeIsActive() && i < 3; i++) {
                lift.gotoLevel(Lift.LiftLevel.Floor, true, null, false);
                drive.goToLocation(coneLocation, 1, 0.2, coneLocation.angle);
                lift.grabber(true); //(Changed to true)
                final int tickDiff = 30;
                lift.gotoLevel(Lift.LiftLevel.Second, -(i * tickDiff), true, null, false);

                drive.goToLocation(midJunction, 1, 0.2, midJunction.angle);
                lift.grabber(false);

            }
        }

        // TODO: Parking in the right place... (OpenCV)
        if (auto._isParking) gotoParkingPosition(parkingPosition);
//        if (auto == Auto.PARK) drive.turn(-179, 0.5);

        lift.resetJoint();
    }

    public void placeFirstCone(ConeJunction coneJunction) {
        //drive.goTo(drive.getPosX(), 1,0.5, -100, 0.15, 3); // goes to high junction position
//        drive.goTo(drive.getPosX(), 1.4,0.5, -100, 0.15, 3); // goes to high junction position
//        opMode.sleep(500);
        drive.goTo(drive.getPosX(), 1.65, 0.6, drive.getHeading(), 0.07, 0);
        drive.goTo(drive.getPosX(), 1.5, 0.3, drive.getHeading(), 0.07, 0);
        Location conePosition = new Location(highJunction);
        switch (coneJunction) {
            case mid:
                lift.gotoLevel(Lift.LiftLevel.Second, true, null);
                opMode.sleep(300);
                drive.turnTo(0, 0.3);
                conePosition = midJunction;
                break;
            case high:
                lift.gotoLevel(Lift.LiftLevel.ThirdAUTO, true, null);
                opMode.sleep(300);
                break;
            case safe:
                lift.gotoLevel(Lift.LiftLevel.ThirdAUTO, true, null);
                opMode.sleep(300);
                conePosition = highJunctionSafe;
                break;
        }
        drive.goToLocation(new Location(drive.getPosX(), conePosition.y - 0.07, drive.getHeading()), 0.5, 0.01, 0);
        drive.goToLocation(conePosition, 0.5, drive.getHeading(), 0.01, 0);

        opMode.sleep(500);
        lift.grabber(false);
        opMode.sleep(300);
    }

    public void placeFirstHighCone() {
        SleevePipeline.ParkingLocation loc = pipelineSleeve.getParkingLocation();
        if (loc == SleevePipeline.ParkingLocation.One) this.parkingPosition = ParkingPosition.one;
        else if (loc == SleevePipeline.ParkingLocation.Two)
            this.parkingPosition = ParkingPosition.two;
        else this.parkingPosition = ParkingPosition.three;

        if (loc == SleevePipeline.ParkingLocation.One) opMode.telemetry.addLine("1");
        else if (loc == SleevePipeline.ParkingLocation.Two) opMode.telemetry.addLine("2");
        else if (loc == SleevePipeline.ParkingLocation.Three) opMode.telemetry.addLine("3");
        opMode.telemetry.update();
        double heading = drive.getHeading();
        drive.goTo(drive.getPosX(), 1.55, 0.6, heading, 0.05, 0);
        lift.gotoLevel(Lift.LiftLevel.ThirdAUTO, true, null);
        drive.goTo(drive.getPosX(), 1.45, 0.3, heading, 0.07, 0);
        drive.goToLocation(highJunction, 0.5, heading, 0.01, 0);

        opMode.sleep(250);
        lift.gotoLevel(Lift.LiftLevel.Second, false, null);
        opMode.sleep(300);
        lift.grabber(false);
        opMode.sleep(300);
        drive.goToLocation(new Location(drive.getPosX(), drive.getPosY() - 0.08, drive.getHeading()), 0.3, 0.07, 0);
        lift.gotoLevel(Lift.LiftLevel.coneStack, false, null);
    }

    public void placeFirstMidCone() {
        SleevePipeline.ParkingLocation loc = pipelineSleeve.getParkingLocation();
        if (loc == SleevePipeline.ParkingLocation.One) this.parkingPosition = ParkingPosition.one;
        else if (loc == SleevePipeline.ParkingLocation.Two)
            this.parkingPosition = ParkingPosition.two;
        else this.parkingPosition = ParkingPosition.three;

        if (loc == SleevePipeline.ParkingLocation.One) opMode.telemetry.addLine("1");
        else if (loc == SleevePipeline.ParkingLocation.Two) opMode.telemetry.addLine("2");
        else if (loc == SleevePipeline.ParkingLocation.Three) opMode.telemetry.addLine("3");
        opMode.telemetry.update();
        double heading = drive.getHeading();
        drive.goTo(drive.getPosX(), 1.65, 0.6, heading, 0.07, 0);
        lift.gotoLevel(Lift.LiftLevel.Second, true, null);
        drive.goTo(drive.getPosX(), 1.45, 0.3, heading, 0.07, 0);
        drive.goToLocation(midJunction, 0.5, heading, 0.01, 0);

        opMode.sleep(150);
        lift.gotoLevel(Lift.LiftLevel.Second, false, null);
        opMode.sleep(300);
        lift.grabber(false);
        opMode.sleep(500);
        drive.goToLocation(new Location(drive.getPosX(), drive.getPosY() - 0.08, drive.getHeading()), 0.3, 0.07, 0);
    }

    public void goToParkingPosition() {
        gotoParkingPosition(parkingPosition);
    }

    public void run2() {
        gotoParkingPosition(parkingPosition);
    }

    public void test() {
        SleevePipeline.ParkingLocation loc = pipelineSleeve.getParkingLocation();
        Log.e("Sci", "camera initialization failed: " + loc);
        opMode.telemetry.addData("camara is on", loc);

        if (loc == SleevePipeline.ParkingLocation.One) {
            opMode.telemetry.addLine("1");
            parkingPosition = ParkingPosition.one;
        } else if (loc == SleevePipeline.ParkingLocation.Two) {
            opMode.telemetry.addLine("2");
            parkingPosition = ParkingPosition.two;
        } else if (loc == SleevePipeline.ParkingLocation.Three) {
            opMode.telemetry.addLine("3");
            parkingPosition = ParkingPosition.three;
        } else {
            opMode.telemetry.addLine("NOTHING");
            opMode.telemetry.update();
            parkingPosition = ParkingPosition.two;
        }
        drive.initRodPipline(camRod, camTower);
        drive.goTo(-0.9, 1.5, 0.3, drive.getHeading(), 0.01, 0);
        drive.turnTo(-135, 0.3);
        drive.zeroOnTarget();
        //
        // placeFirstCone(ConeJunction.mid);
        //placeFirstHighCone();
        // gotoParkingPosition(parkingPosition);
    }

    public void run3() {

        drive.goToLocation(new Location(0, 0.3), 0.4, 0, 0.01, 0);
        opMode.sleep(300);
        drive.turnTo(0, 0.4);

    }

    public void placeCones() {
        SleevePipeline.ParkingLocation loc = pipelineSleeve.getParkingLocation();
        Log.e("Sci", "camera initialization failed: " + loc);
        opMode.telemetry.addData("camara is on", loc);

        if (loc == SleevePipeline.ParkingLocation.One) {
            opMode.telemetry.addLine("1");
            parkingPosition = ParkingPosition.one;
        } else if (loc == SleevePipeline.ParkingLocation.Two) {
            opMode.telemetry.addLine("2");
            parkingPosition = ParkingPosition.two;
        } else if (loc == SleevePipeline.ParkingLocation.Three) {
            opMode.telemetry.addLine("3");
            parkingPosition = ParkingPosition.three;
        } else {
            opMode.telemetry.addLine("NOTHING");
            opMode.telemetry.update();
            parkingPosition = ParkingPosition.two;
        }
        for (int i = 0; opMode.opModeIsActive() && i < 5; i++) {
            placeCone(highJunction);
        }

    }

    public enum ConeJunction {
        mid,
        high,
        safe
    }

//    public void placeCones() {
//
//        lift.gotoLevel(Lift.LiftLevel.cone5, false, null); // goes to the highest cone
//        lift.grabber(false); // opens grabber
//        opMode.sleep(50);
//        drive.goToLocation(new Location(0, coneLocation.y, drive.getHeading()),
//                0.5, 0.1, 3); // goes to robot x, and cone y
//        drive.turnTo(-90, 0.5);
//        drive.goToLocation(new Location(-0.2, coneLocation.y, drive.getHeading()),
//                0.5, 0.1, 3); // goes to robot x, and cone y
//        drive.turnTo(-90, 0.5);
//        drive.goToLocation(new Location(-0.50, drive.getPosY(), -90), 0.5, 0.1, 3); // goes to cone x, and robot y
////        drive.goToLocation(new Location(drive.getPosX() + 0.4, drive.getPosY(), -90), 0.5, 0.1, 3);
//        opMode.sleep(300);
//        lift.grabber(true); // closes grabber
//        opMode.sleep(500);
//        lift.gotoLevel(Lift.LiftLevel.Third, false, null);
//        opMode.sleep(500);
//        lift.toggleFlip(null);
//        drive.goToLocation(new Location(drive.getPosX() + 0.5,
//                        drive.getPosY(),
//                        drive.getHeading()),
//                0.5, 0.1, 3);
//        drive.turnTo(-180, 0.5);
//        opMode.sleep(500);
//        drive.goToLocation(new Location(highJunction.x + 0.1,
//                        highJunction.y,
//                        drive.getHeading()),
//                0.3, 0.1, 3);
//        opMode.sleep(400);
//        lift.grabber(false);
//        opMode.sleep(300);
//        gotoParkingPosition(ParkingPosition.three);
////        lift.gotoLevel(Lift.LiftLevel.Third, false); // goes to max height of elevator
////        drive.turn(90, 0.5);
////        drive.goToLocation(new Location(highJunctionSafe.x, highJunctionSafe.y, 90), 0.5, 0.1, 3); // goes to high junction position
////        lift.toggleFlip(); // changes side of flip motors
////        lift.grabber(false); // opens grabber
//
//    }

    public enum SIDE {
        LEFT(-1), RIGHT(1);

        public int mul;

        SIDE(int mul) {
            this.mul = mul;
        }
    }

    public enum ParkingPosition {
        one(new Location(-1.5, 1.57, 180)),
        two(new Location(-0.9, 1.57, 180)),
        three(new Location(-0.25, 1.57, 180));

        public Location location;

        ParkingPosition(Location location) {
            this.location = location;
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

    public void pita_cycle() {
//        Location highJunction1 = new Location(-0.75, 1.68, -135* startPos.mul); // also park 2
//        Location prePark1 = new Location(-0.90,1.60,-90* startPos.mul);
//        Location coneLocation1 = new Location(-1.5, 1.53, -90*startPos.mul);

        SleevePipeline.ParkingLocation loc = pipelineSleeve.getParkingLocation();
        if (loc == SleevePipeline.ParkingLocation.One)
            this.parkingPosition = ParkingPosition.one;
        else if (loc == SleevePipeline.ParkingLocation.Two)
            this.parkingPosition = ParkingPosition.two;
        else if (loc == SleevePipeline.ParkingLocation.Three)
            this.parkingPosition = ParkingPosition.three;
        else
            this.parkingPosition = ParkingPosition.two;


        if (loc == SleevePipeline.ParkingLocation.One) opMode.telemetry.addLine("after start 1");
        else if (loc == SleevePipeline.ParkingLocation.Two)
            opMode.telemetry.addLine("after start 2");
        else if (loc == SleevePipeline.ParkingLocation.Three)
            opMode.telemetry.addLine("after start 3");
        opMode.telemetry.update();

        drive.initRodPipline(camRod, camTower);
        double heading = drive.getHeading();
        drive.goTo(drive.getPosX(), 1.75, 1, heading, 0.05, 0, true);
        lift.gotoLevel(Lift.LiftLevel.ThirdAUTO, true, null);
        drive.goTo(drive.getPosX(), 1.45, 0.8, highJunction1.angle, 0.05, 0);
        opMode.sleep(150);
        drive.goTo(highJunction1.x - 0.05 * startPos.mul, highJunction1.y - 0.05, 0.7, highJunction1.angle, 0.03, 0);
        drive.zeroOnTarget();

        //opMode.sleep(300);
        lift.grabber(false);
        opMode.sleep(100);
//		Log.d("Sci", String.format("prePark.x: %f", prePark1.x) );

        for (int i = 0; opMode.opModeIsActive() && i < 2; i++) {
            drive.goToLocation(prePark1, 1, 0.05, 0);
            lift.gotoLevel(Lift.LiftLevel.coneStack, -i * 100, true, null, false);
            drive.turnTo(-90 * startPos.mul, 1);
            lift.grabber(false);
            drive.goToLocation(coneLocation1, 0.8, 0.05, 0, true);
            lift.grabber(true); // grab the cone
            opMode.sleep(600);
            lift.gotoLevel(Lift.LiftLevel.Second, false, null);
            opMode.sleep(150);
            drive.goTo(preStack.x, preStack.y, 1, preStack.angle, 0.07, 0, true);
            lift.gotoLevel(Lift.LiftLevel.ThirdAUTO, true, null);
            opMode.sleep(250);

            drive.goTo(highJunction1.x, highJunction1.y, 0.7, highJunction1.angle, 0.04, 0);
            drive.turnTo(highJunction1.angle, 0.7);
            drive.zeroOnTarget();
            opMode.sleep(150);
            lift.grabber(false);
            //lift.gotoLevel(Lift.LiftLevel.Second, false, null);
            opMode.sleep(350);
        }
        drive.goToLocation(prePark1, 1, 180, 0.05, 0);
        opMode.sleep(200);
        lift.gotoLevel(Lift.LiftLevel.Floor, false, null);
//        lift.rotate(true);
        opMode.telemetry.addData("Going to", "parking position: x=" + parkingPosition.location.x + " y=" + parkingPosition.location.y);
        opMode.telemetry.update();
        double rightShift = startPos == StartPos.RIGHT ? 1.80 : 0;
        drive.goTo(parkingPosition.location.x + rightShift, parkingPosition.location.y, 1, 180, 0.05, 0);
        drive.turnTo(180, 0.8);
        opMode.telemetry.addData("heding", drive.getHeading());
        //drive.turnTo(180, 0.5);
    }


    public void pita_new() {
        Location highJunction2 = new Location(-0.75, 1.68, -135 * startPos.mul); // also park 2
        Location coneLocation2 = new Location(-1.5, 1.53, -90 * startPos.mul);

        SleevePipeline.ParkingLocation loc = SetupCamera();


        drive.initRodPipline(camRod, camTower);
        double heading = drive.getHeading();
        drive.goTo(drive.getPosX(), 1.20, 1, heading, 0.10, 0, true);
        lift.gotoLevel(Lift.LiftLevel.ThirdAUTO, true, null);
        drive.goTo(drive.getPosX(), 1.80, 1, heading, 0.05, 0);
        opMode.sleep(150);
        lift.grabber(false);
        opMode.sleep(100);
//		Log.d("Sci", String.format("prePark.x: %f", prePark1.x) );
    }

    /**
     * New Autonomous, uses the cones param to determine the amount of cones
     *
     * @param cones Amount of cones
     */
    public void newRun(int cones) {
        Location highJunction2 = new Location(-0.70, 1.53, -135 * startPos.mul); // also park 2
        Location coneLocation2 = new Location(-1.55, 1.53, -90 * startPos.mul);

        SleevePipeline.ParkingLocation loc = SetupCamera();


        drive.initRodPipline(camRod, camTower);
        double heading = drive.getHeading();
        lift.gotoLevel(Lift.LiftLevel.Second, false, null);
        drive.goTo(drive.getPosX(), 1.27, 1, heading, 0.10, 0, true);
        opMode.sleep(500);
        lift.armGotoPosition(Lift.ArmPos.Right);
        opMode.sleep(500);
        lift.grabber(true);
        opMode.sleep(700);
        lift.ResetArmPos();
        opMode.sleep(700);
        drive.goTo(drive.getPosX(), 1.53, 1, heading, 0.10, 0, true);
        drive.turnTo(90, 0.5);
        for (int i = 0; i < cones; i++) {
            lift.gotoLevel(Lift.LiftLevel.coneStack, false, null);
            drive.goTo(coneLocation2.x, coneLocation2.y, 1, 90, 0.10, 0, true);
            opMode.sleep(500);
            lift.grabber(false);
            opMode.sleep(400);
            lift.gotoLevel(Lift.LiftLevel.Third, false, null);
            opMode.sleep(500);
            drive.goTo(highJunction2.x, highJunction2.y, 0.7, 90, 0.10, 0, true);
            lift.armGotoPosition(Lift.ArmPos.Right);
            opMode.sleep(500);
            lift.grabber(true);
            opMode.sleep(700);
            lift.armGotoPosition(Lift.ArmPos.Middle);
            opMode.sleep(500);
        }
        opMode.sleep(1000);
        drive.turnTo(heading, 0.5);
        opMode.sleep(500);
        drive.goTo(-0.9, 1.53, 1, heading, 0.10, 0, true);
        opMode.sleep(20000);
        /*
	PESCUDO CODE
	-----------
	turn on cam and get parking pos
	go to above middle to prevent cone from interrupting
	turn -90 deg towards the cone stack
	elevator to 3rd height
	drive to (-0.6, 1.5)
	arm to right
	release cone to right pole
	return arm to middle
	elevator the 5th cone height
	[loop]
		go to cone stack
		catch cone
		go back
		elevator to matching height
		arm to right
		release cone
		return arm to middle
		elevator to cone height
	[end loop]
	park
	 */
    }

    SleevePipeline.ParkingLocation SetupCamera() {
        SleevePipeline.ParkingLocation loc = pipelineSleeve.getParkingLocation();
        if (loc == SleevePipeline.ParkingLocation.One)
            this.parkingPosition = ParkingPosition.one;
        else if (loc == SleevePipeline.ParkingLocation.Two)
            this.parkingPosition = ParkingPosition.two;
        else if (loc == SleevePipeline.ParkingLocation.Three)
            this.parkingPosition = ParkingPosition.three;
        else
            this.parkingPosition = ParkingPosition.two;


        if (loc == SleevePipeline.ParkingLocation.One) opMode.telemetry.addLine("after start 1");
        else if (loc == SleevePipeline.ParkingLocation.Two)
            opMode.telemetry.addLine("after start 2");
        else if (loc == SleevePipeline.ParkingLocation.Three)
            opMode.telemetry.addLine("after start 3");
        opMode.telemetry.update();


        return loc;
    }

    void RepositionLocationCone(double turnToDeg) {
        drive.initRodPipline(camRod, camTower);
        double heading = drive.getHeading();
        drive.goTo(drive.getPosX(), 1.75, 1, heading, 0.05, 0, true);
        lift.gotoLevel(Lift.LiftLevel.ThirdAUTO, true, null);
        drive.goTo(drive.getPosX(), 1.45, 0.8, turnToDeg, 0.05, 0);
        opMode.sleep(150);
    }
}
