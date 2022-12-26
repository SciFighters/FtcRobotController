package org.firstinspires.ftc.teamcode.power_play.Autonomous;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.power_play.util.DriveClass;
import org.firstinspires.ftc.teamcode.power_play.util.Lift;
import org.firstinspires.ftc.teamcode.power_play.util.Location;

public class AutoFlow {
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
        SHORT(1,true),
        LONG(2,true),
        PARK(3,true),
        FULL(4,true),
        CYCLING(5,true);

        public int _value;
        public boolean _isParking;
        Auto(int _value, boolean isParking) {
            this._value = _value;
            this._isParking = isParking;
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
    Location park2 = new Location(highJunction);
    Location park3 = new Location(coneLocation);
    //endregion


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
//        drive = new DriveClass(opMode, DriveClass.ROBOT.CONSTANTIN, new Location(0.5, 0.5));
        drive.init(opMode.hardwareMap);
        opMode.telemetry.update();

    }
    private void parkAtConeLocation() {

    }


    public void run() {
//Autonomous starts
        if (auto == Auto.FULL) {
            lift.gotoLevel(Lift.LiftLevel.Third);
            for (int i = 0; i < 4; i++) { // Going to put 4 cones
                drive.goToLocation(highJunction, 1, 0.2, 0);
                lift.grabber(false); // changed to false
                lift.gotoLevel(Lift.LiftLevel.Floor); // TODO: check if the height right? (cone pile)
                drive.goToLocation(coneLocation, 1, 0.2, 0); // TODO: change cone location (1.5, 1.5 ? )
                lift.grabber(true); //(Changed to true) TODO: check validity of grabber ability
                lift.gotoLevel(Lift.LiftLevel.Third);
            }


        }
        if (auto==auto.LONG){ //backup, less points
            lift.gotoLevel(Lift.LiftLevel.Third);
            drive.goToLocation(highJunctionSafe,1,0.2,highJunctionSafe.angle);
            lift.grabber(false); //(Changed to false) TODO: check
            for (int i = 0; i < 3; i++) {
                lift.gotoLevel(Lift.LiftLevel.Floor);
                drive.goToLocation(coneLocation,1,0.2,coneLocation.angle);
                lift.grabber(true); //(Changed to true)

                lift.gotoLevel(Lift.LiftLevel.Second);
                drive.goToLocation(medJunction,1,0.2,medJunction.angle);
                lift.grabber(false);

            }
        }


        // TODO: Parking in the right place... (OpenCV)
        if(auto._isParking) parkAtConeLocation();

        //

    }

}


