// Drive Class
// CENTERSTAGE 2023 - 2024
package org.firstinspires.ftc.teamcode.centerstage.Systems;

import android.util.Log;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.canvas.Canvas;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.RobotLog;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.robotcore.external.navigation.Position;
import org.firstinspires.ftc.robotcore.external.navigation.Velocity;
import org.firstinspires.ftc.teamcode.centerstage.util.ECSSystem.Component;
import org.firstinspires.ftc.teamcode.centerstage.util.Location;
import org.firstinspires.ftc.teamcode.centerstage.util.Util;
import org.firstinspires.ftc.teamcode.power_play.util.IMU_Integrator;
import org.firstinspires.ftc.teamcode.power_play.util.RodLine;
import org.opencv.core.Point;
import org.openftc.easyopencv.OpenCvWebcam;

public class DriveClass extends Component {
    public static final int USE_ENCODERS = 1 << 0;
    public static final int USE_BRAKE = 1 << 1;
    public static final int USE_DASHBOARD_FIELD = 1 << 2;
    public final int MAX_IDLE_BREAK = 20;
    final double tile = 0.6;
    private final BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
    private final boolean useEncoders;
    private final boolean useBrake;
    private final boolean useDashboardField;
    private final boolean fieldOriented = true;
    private final RodLine rodPipline = new RodLine().useYellow();
    private final RodLine towerPipline = new RodLine();
    private final ROBOT robotType;
    private final Location startingPosition;
    public boolean busy;
    volatile public DcMotorEx fl = null;
    DriveMode mode;
    ElapsedTime timer = new ElapsedTime();
    volatile private DcMotorEx fr = null;
    volatile private DcMotorEx bl = null;
    volatile private DcMotorEx br = null;
    private BNO055IMU imu = null;
    private double angleOffset = 0;
    private int fl_startPos = 0;
    private int fr_startPos = 0;
    private int bl_startPos = 0;
    private int br_startPos = 0;
    private double forwardTicksPerMeter;
    private double strafeTicksPerMeter;
    private DistanceSensor leftDistanceSensor, rightDistanceSensor;

    public DriveClass(ROBOT robotType, Location startingPosition, int flags, DriveMode mode) {
        this.robotType = robotType;
        this.startingPosition = startingPosition;
        this.mode = mode;


        if (robotType == ROBOT.JACCOUSE) {
            this.forwardTicksPerMeter = 1562.5;
            this.strafeTicksPerMeter = 1645.83;
        } else if (robotType == ROBOT.SCORPION) {
            this.forwardTicksPerMeter = 2455;
            this.strafeTicksPerMeter = 2587;
        } else if (robotType == ROBOT.COBALT) {
            this.forwardTicksPerMeter = 1753;
            this.strafeTicksPerMeter = 2006;
        } else if (robotType == ROBOT.GLADOS) {
            this.forwardTicksPerMeter = 1000;
            this.strafeTicksPerMeter = 1110;
        }
        switch (robotType) {
            case JACCOUSE:
                this.forwardTicksPerMeter = 1562.5;
                this.strafeTicksPerMeter = 1645.83;
                break;
            case COBALT:
                this.forwardTicksPerMeter = 1753;
                this.strafeTicksPerMeter = 2006;
                break;
            case SCORPION:
                this.forwardTicksPerMeter = 2455;
                this.strafeTicksPerMeter = 2587;
                break;
            case CONSTANTIN:
                this.forwardTicksPerMeter = 1753;
                this.strafeTicksPerMeter = 2006;
                break;
            case GLADOS:
                this.forwardTicksPerMeter = 1000;
                this.strafeTicksPerMeter = 1110;
            default:
                this.forwardTicksPerMeter = 1753;
                this.strafeTicksPerMeter = 2006;

        }

        this.useEncoders = (flags & USE_ENCODERS) != 0;
        this.useBrake = (flags & USE_BRAKE) != 0;
        this.useDashboardField = (flags & USE_DASHBOARD_FIELD) != 0;

//        if (this.useDashboardField) {
//            this.robot_pathx = new ArrayList<>();
//            this.robot_pathy = new ArrayList<>();
//
//            this.robot_pathx.add(startingPosition.x * meters_to_inches);
//            this.robot_pathy.add(startingPosition.y * meters_to_inches);
//        }
    }

    public void initRodPipline(OpenCvWebcam rod, OpenCvWebcam tower) {
        rod.setPipeline(rodPipline);
        if (tower != null) tower.setPipeline(towerPipline);

    }

    @Override
    public void init() {
        RobotLog.d("motors init start");

        //region get from hardwareMap
        fl = hardwareMap.get(DcMotorEx.class, "fl");
        fr = hardwareMap.get(DcMotorEx.class, "fr");
        bl = hardwareMap.get(DcMotorEx.class, "bl");
        br = hardwareMap.get(DcMotorEx.class, "br");
        rightDistanceSensor = hardwareMap.get(DistanceSensor.class, "rearRightDistanceSensor");
        leftDistanceSensor = hardwareMap.get(DistanceSensor.class, "rearLeftDistanceSensor");
        //endregion get from hardwareMap

        //region setDirection
        fl.setDirection(DcMotorEx.Direction.REVERSE);
        fr.setDirection(DcMotorEx.Direction.FORWARD);
        bl.setDirection(DcMotorEx.Direction.REVERSE);
        br.setDirection(DcMotorEx.Direction.FORWARD);
        //endregion setDirection

        fl.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        fr.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        bl.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        br.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);

        //region setMode
        if (useEncoders) {
            fl.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
            fr.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
            bl.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
            br.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
        } else {
            fl.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);
            fr.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);
            bl.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);
            br.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);
        }
        //endregion setMode

        //region setZeroPowerBehavior
        if (useBrake) {
            fl.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
            fr.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
            bl.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
            br.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        }
        //endregion setZeroPowerBehavior

        robot.telemetry.addData("use encoders", this.useEncoders);
        robot.telemetry.addData("use brake", this.useBrake);
        robot.telemetry.addData("use dash", this.useDashboardField);
        robot.telemetry.update();
        RobotLog.d("imu init start");

        initIMU(hardwareMap);
    }

    private void initIMU(HardwareMap hw) {
        imu = hw.get(BNO055IMU.class, "imu 1");

        parameters.angleUnit = BNO055IMU.AngleUnit.DEGREES;
        parameters.accelUnit = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        RobotLog.d("imu params init start");
        parameters.accelerationIntegrationAlgorithm = new IMU_Integrator(imu, hw, forwardTicksPerMeter, strafeTicksPerMeter, !this.useDashboardField, this.mode.origin, this.mode.direciton, startingPosition.angle);
        RobotLog.d("imu init");
        imu.initialize(parameters);
        RobotLog.d("imu init finished");

        robot.telemetry.addData("Gyro", "calibrating...");
        robot.telemetry.addData("Integrator dashboard", this.useDashboardField);
        // robotType.telemetry.update();

        ElapsedTime timer = new ElapsedTime();
        timer.reset();
        while (!imu.isGyroCalibrated() && !robot.isStopRequested() && timer.seconds() < 5) {
            robot.sleep(50);
        }
        if (imu.isGyroCalibrated()) {
            robot.telemetry.addData("Gyro", "Done Calibrating");
            RobotLog.d("Gyro done init");

        } else {
            robot.telemetry.addData("Gyro", "Gyro/IMU Calibration Failed");
            RobotLog.d("Gyro failed init" + " " + imu.isGyroCalibrated() + " " + imu.isAccelerometerCalibrated() + " " + imu.isMagnetometerCalibrated());
        }

        imu.startAccelerationIntegration(new Position(DistanceUnit.METER, this.startingPosition.x, this.startingPosition.y, 0, 0), new Velocity(), 2);

        // robotType.telemetry.update();

        RobotLog.d("IMU status: %s", imu.getSystemStatus().toShortString());
        RobotLog.d("IMU calibration status: %s", imu.getCalibrationStatus().toString());
    }

    public double getDistanceRightSensorDistance() {
        return rightDistanceSensor.getDistance(DistanceUnit.CM);
    }

    public double getDistanceLeftSensorDistance() {
        return leftDistanceSensor.getDistance(DistanceUnit.CM);
    }

    public double getImuDistance(Position target) {
        Position current = imu.getPosition();
        double dx = current.x - target.x;
        double dy = current.y - target.y;
        double sqrt = Math.pow(dy, 2) + Math.pow(dx, 2);
        return Math.sqrt(sqrt);
    }

    public void setPower(double forward, double turn, double strafe) {
        fl.setPower(forward + turn + strafe);
        fr.setPower(forward - turn - strafe);
        bl.setPower(forward + turn - strafe);
        br.setPower(forward - turn + strafe);
    }

    public void setPowerOriented(double y, double x, double turn, boolean fieldOriented) {
        if (!fieldOriented) {
            setPower(y, turn, x);  // No field oriented  (=> Robot oriented)
        } else {
            double phiRad = (-getHeading() + angleOffset) / 180 * Math.PI;
            double forward = y * Math.cos(phiRad) - x * Math.sin(phiRad);
            double strafe = y * Math.sin(phiRad) + x * Math.cos(phiRad);
            setPower(forward, turn, strafe);
        }

//		robotType.telemetry.addData("Front","left/right: %d, %d", fl.getCurrentPosition(), fr.getCurrentPosition());
//		robotType.telemetry.addData("Back","left/right: %d, %d", bl.getCurrentPosition(), br.getCurrentPosition());
    }

    public void stopPower() {
        setPower(0, 0, 0);
        busy = false;
    }

    public void resetOrientation(double angle) {
        imu.initialize(parameters);
        angleOffset = angle;
    }

    public void resetHeading(double angle) {
        resetOrientation(angle);
    }

    public double getHeading() {
        Orientation orientation = imu.getAngularOrientation();
        return (-orientation.firstAngle + startingPosition.angle) % 360;
    }

    public double getDeltaHeading(double target) {
        double currentAngle = getHeading();
        double delta = (target - currentAngle) % 360;

        if (delta < -180) {
            delta = delta + 360;
        }
        if (delta > 180) {
            delta = delta - 360;
        }
        return delta;
    }

    public double getForwardDistance() {
        double fl_tick = fl.getCurrentPosition() - fl_startPos;
        double fr_tick = fr.getCurrentPosition() - fr_startPos;
        double bl_tick = bl.getCurrentPosition() - bl_startPos;
        double br_tick = br.getCurrentPosition() - br_startPos;
        double fl_dist = fl_tick / forwardTicksPerMeter;
        double fr_dist = fr_tick / forwardTicksPerMeter;
        double bl_dist = bl_tick / forwardTicksPerMeter;
        double br_dist = br_tick / forwardTicksPerMeter;
        return (bl_dist + br_dist + fr_dist + fl_dist) / 4;
    }

    public double getPosY() {
        return imu.getPosition().y;
    }

    public double getAbsolutesPosY() {
        double fl_tick = fl.getCurrentPosition();
        double fr_tick = fr.getCurrentPosition();
        double bl_tick = bl.getCurrentPosition();
        double br_tick = br.getCurrentPosition();
        double fl_dist = fl_tick / forwardTicksPerMeter;
        double fr_dist = fr_tick / forwardTicksPerMeter;
        double bl_dist = bl_tick / forwardTicksPerMeter;
        double br_dist = br_tick / forwardTicksPerMeter;
        return (bl_dist + br_dist + fr_dist + fl_dist) / 4;
    }

    public double getStrafeDistance() {
        double fl_tick = fl.getCurrentPosition() - fl_startPos;
        double fr_tick = fr.getCurrentPosition() - fr_startPos;
        double bl_tick = bl.getCurrentPosition() - bl_startPos;
        double br_tick = br.getCurrentPosition() - br_startPos;

        double flDist = fl_tick / strafeTicksPerMeter;
        double frDist = fr_tick / strafeTicksPerMeter;
        double blDist = bl_tick / strafeTicksPerMeter;
        double brDist = br_tick / strafeTicksPerMeter;
        return (-blDist + brDist - frDist + flDist) / 4;
    }

    public double getPosX() {
        return imu.getPosition().x;
    }

    public double getAbsolutesPosX() {
        double fl_tick = fl.getCurrentPosition();
        double fr_tick = fr.getCurrentPosition();
        double bl_tick = bl.getCurrentPosition();
        double br_tick = br.getCurrentPosition();

        double flDist = fl_tick / strafeTicksPerMeter;
        double frDist = fr_tick / strafeTicksPerMeter;
        double blDist = bl_tick / strafeTicksPerMeter;
        double brDist = br_tick / strafeTicksPerMeter;
        return (-blDist + brDist - frDist + flDist) / 4;
    }

    public void printWheelsPosition() {
        robot.telemetry.addData("fl", fl.getCurrentPosition());
        robot.telemetry.addData("fr", fr.getCurrentPosition());
        robot.telemetry.addData("bl", bl.getCurrentPosition());
        robot.telemetry.addData("br", br.getCurrentPosition());

        robot.telemetry.addData("average", (fl.getCurrentPosition() + fr.getCurrentPosition() + bl.getCurrentPosition() + br.getCurrentPosition()) / 4);

        robot.telemetry.addData("forwardDist", getForwardDistance());
        robot.telemetry.addData("strafeDist", getStrafeDistance());
    }

    public void resetPosition() {
        fl_startPos = fl.getCurrentPosition();
        fr_startPos = fr.getCurrentPosition();
        bl_startPos = bl.getCurrentPosition();
        br_startPos = br.getCurrentPosition();
        robot.telemetry.addData("RESET POSITION !!!!!!", 0);
    }

    public void turn(double deg, double power) {
        busy = true;
        double targetAngle = getHeading() + deg; // zeroAngle
        turnTo(targetAngle, power);
    }

    public void turnTo(double targetAngle, double targetPower) {
        double delta = getDeltaHeading(targetAngle);
        double s = (delta < 0) ? -1 : 1;
        while ((delta * s > 0) && robot.opModeIsActive()) {

            delta = getDeltaHeading(targetAngle);
            double gain = 0.02;
            double power = gain * delta * targetPower;
            if (Math.abs(power) < 0.1)
                power = 0.1 * Math.signum(power);

            setPower(0, power, 0);

//            robot.telemetry.addData("target", targetAngle);
//            robot.telemetry.addData("current", getHeading());
//            robot.telemetry.addData("delta", delta);
//            robot.telemetry.addData("power", power);
            robot.telemetry.update();
        }
        stopPower();
    }

    public double goToLocation(Location location, double power, double targetHeading, double tolerance, double timeout) {
        return goTo(location.x, location.y, power, targetHeading, tolerance, timeout, false);
    }

    public double goToLocation(Location location, GotoSettings settings, Runnable midwayAction) {
        return goToAndOperate(location, settings, midwayAction);
    }

    private double goToAndOperate(Location location, GotoSettings settings, Runnable midwayAction) {
        Thread thread = Util.loopAsync(midwayAction, robot);
        double goToResult = goToLocation(location, settings);
        thread.interrupt();
        return goToResult;
    }

    public double goToLocation(Location location, double power, double targetHeading, double tolerance, double timeout, boolean noSlowdown) {
        return goTo(location.x, location.y, power, targetHeading, tolerance, timeout, noSlowdown);
    }

    public double goToLocationOnAxis(Location location, double power, double tolerance, double timeout, Axis axis, boolean superSpeedX, boolean superSpeedY) {
        switch (axis) {
            case x:
                goTo(location.x, getPosY(), power, location.angle, 0.15, timeout, superSpeedX);
            case y:
                goTo(getPosX(), location.y, power, location.angle, 0.15, timeout, superSpeedY);
        }
        return goToLocation(location, power, tolerance, timeout);
    }

    public double goToLocationOnAxis(Location location, GotoSettings settings, Axis axis) {

        return goToLocationOnAxis(location, settings.power, settings.tolerance, settings.timeout, axis, settings.noSlowdown, settings.noSlowdown);
    }

    public double goToLocationOnAxis(Location location, double power, double tolerance, double timeout, Axis axis) {
        return this.goToLocationOnAxis(location, power, tolerance, timeout, axis, false, false);
    }

    public double goToLocation(Location location, double power, double tolerance, double timeout) {
        return goTo(location.x, location.y, power, location.angle, tolerance, timeout, false);
    }

    public double goTo(double x, double y, double targetPower, double targetHeading, double tolerance, double timeout) {
        return this.goTo(x, y, targetPower, targetHeading, tolerance, timeout, false);
    }

    public double goToLocation(Location location, double power, double tolerance, double timeout, boolean superSpeed) {
        return goTo(location.x, location.y, power, location.angle, tolerance, timeout, superSpeed);
    }

    public double goToLocation(Location location, GotoSettings settings) {
        return goTo(location.x, location.y, settings.power, location.angle, settings.tolerance, settings.timeout, settings.noSlowdown);
    }

    public double goToLocation(Location location, double targetHeading, GotoSettings settings) {
        return goToLocation(new Location(location.x, location.y, targetHeading), settings);
    }

    public double goTo(double x, double y, double targetPower, double targetHeading, double tolerance, double timeout, boolean superSpeed) {
        int goToIdle = 0; //if not moving
        int stuckTries = 0;
        boolean isCheckingIdle = false;

        double lastX = 0;
        double lastY = 0;

        double currentX = getPosX();
        double currentY = getPosY();
        double deltaX = x - currentX;
        double deltaY = y - currentY;
        double startX = currentX;
        double startY = currentY;
        double remainDist = 0;


        Location returnToPosAfterStuck = new Location(startX, startY);
        //double totalDist = Math.sqrt(deltaX*deltaX + deltaY*deltaY);
        double totalDist = Math.hypot(deltaX, deltaY);
        double currentDist = 0;

        ElapsedTime timer = new ElapsedTime();

        robot.telemetry.addData("goto x", x);
        robot.telemetry.addData("goto y", y);
        robot.telemetry.addData("goto delta x", deltaX);
        robot.telemetry.addData("goto delta y", deltaY);
        robot.telemetry.addData("goto hypocampus total dist", totalDist);
        robot.telemetry.update();
        // timer delta variables
        double currentTime = System.nanoTime();
        double lastTime = System.nanoTime();
        while (robot.opModeIsActive() && (currentDist < (totalDist - tolerance)) && !robot.isStopRequested() && stuckTries < 15) {

            double power = targetPower;

            currentX = getPosX();
            currentY = getPosY();
            deltaX = currentX - startX;
            deltaY = currentY - startY;
            currentDist = Math.hypot(deltaY, deltaX); // distance moved from start position.
            deltaX = x - currentX;
            deltaY = y - currentY;
            remainDist = Math.hypot(deltaY, deltaX);  // distance left to target.

            //double leftDist = totalDist - currentDist;
            double minPower = 0.2;

            double acclGain = superSpeed ? 3.0 : 1.5;
            double acclPower = currentDist * acclGain + minPower;

            double RVy = deltaY / remainDist;  // y velocity ratio
            double RVx = deltaX / remainDist;  // x velocity ratio


            if (acclPower < power) {
                power = acclPower;
            }

            double breakGain = superSpeed ? 1.0 : 0.5;
            double breakPower = remainDist * breakGain + minPower;

            if (breakPower < power) { //
                power = breakPower;
            }

            double headingErr = getDeltaHeading(targetHeading) / 180;
            double headingGain = Math.max(0.6, -0.25 * totalDist + 0.95); // y = -0.25x + 0.95
            double correction = headingGain * headingErr;
            double Vy = RVy * power;
            double Vx = RVx * power;

            //setPowerOriented(Vy, Vx, correction, true);
            double phiRad = -getHeading() / 180 * Math.PI;
            double forward = Vy * Math.cos(phiRad) - Vx * Math.sin(phiRad);
            double strafe = Vy * Math.sin(phiRad) + Vx * Math.cos(phiRad);
            setPower(forward, correction, strafe);


            robot.telemetry.addData("distance", "%2.3f, %2.3f", currentDist, remainDist);
            robot.telemetry.addData("Abs Pos", "X,Y %2.3f, %2.3f", getAbsolutesPosX(), getAbsolutesPosY());
            robot.telemetry.addData("> current", "X,Y: %2.3f, %2.3f", currentX, currentY);
            robot.telemetry.addData("goto delta", " x,y: %2.3f, %2.3f", deltaX, deltaY);
            robot.telemetry.addData("goto velos", "f,s: %2.3f, %2.3f", Vy, Vx);
            robot.telemetry.addData("heading ", getHeading());
            robot.telemetry.addData("heading error", headingErr);
            robot.telemetry.addData("power", power);
            robot.telemetry.update();

            if ((timeout != 0 && timeout <= timer.seconds())) break;

            // timer delta
            currentTime = System.nanoTime();
            double timeDelta = ((currentTime - lastTime) / Math.pow(10, 9)); // timer delta in seconds
            lastTime = currentTime;
            // Idle checker
            double velocityRange = 0.0001;
            double dx = lastX - currentX;
            double dy = lastY - currentY;
            double velocity = Math.hypot(dx, dy) / timeDelta;
            //Log.d("velocity", String.valueOf(velocity));
            if (remainDist < 0.25 && Math.abs(velocity) < velocityRange) {
                goToIdle += 1;
            }

            if (Math.abs(currentX - lastX) < 0.02 && Math.abs(velocity) < velocityRange) {
                stuckTries += 1;
                returnToPosAfterStuck = new Location(getPosX(), getPosY());
            }

            lastX = currentX;
            lastY = currentY;
            if (goToIdle >= MAX_IDLE_BREAK) {
                remainDist = -1;
                break;
            }
        }
        setPower(0, 0, 0);
        return remainDist;
    }

    public void drive(double forward, double sideward, double targetPower, double targetAngle) {
        drive(forward, sideward, targetPower, targetAngle, true);
    }

    public void drive(double forward, double sideward, double targetPower, double targetAngle, boolean fieldOriented) {
        drive(forward, sideward, targetPower, targetAngle, fieldOriented, 0.02);
    }

    public void drive(double forward, double sideward, double targetPower, double targetAngle, boolean fieldOriented, double tolerance) {
        double sf = (forward < 0) ? -1 : 1;
        double ss = (sideward < 0) ? -1 : 1;
        double c = Math.sqrt(sideward * sideward + forward * forward);
        double RVf = forward / c;
        double RVs = sideward / c;
        final double minPower = 0.2;

        resetPosition();
        timer.reset();

        while (robot.opModeIsActive() && (RVf != 0) || (RVs != 0)) {

            if (getForwardDistance() * sf > forward * sf - tolerance) {
                RVf = 0;
            }
            if (getStrafeDistance() * ss > sideward * ss - tolerance) {
                RVs = 0;
            }
            double power = targetPower;

            double deltaForward = forward - getForwardDistance();
            double deltaStrafe = sideward - getStrafeDistance();

            double deltaC = Math.sqrt(deltaStrafe * deltaStrafe + deltaForward * deltaForward);
            double lengthC = c - deltaC;

            double acclGain = 2;
            double acclPower = lengthC * acclGain + minPower;

            if (acclPower + minPower < power) {
                power = acclPower;
            }

            double breakgain = 0.9;
            double breakPower = deltaC * breakgain + minPower;

            if (breakPower < power) {
                power = breakPower;
            }

            power = Math.max(power, minPower);

            double err = getDeltaHeading(targetAngle);
            double gain = 0.040;
            double correction = gain * err;
            double Vf = RVf * power;
            double Vs = RVs * power;

            setPowerOriented(Vf, Vs, correction, fieldOriented);

            robot.telemetry.addData("timer", timer.milliseconds());
            //position Telemetry:
            robot.telemetry.addData("x position:", getAbsolutesPosX());
            robot.telemetry.addData("y position:", getAbsolutesPosY());


            robot.telemetry.addData("delta forward:", deltaForward);
            robot.telemetry.addData("speed forward:", Vf);
//			robotType.telemetry.addData("delta strafe:", deltaStrafe);
            robot.telemetry.addData("speed strafe:", Vs);
            robot.telemetry.addData("power:", power);
            robot.telemetry.addData("current pos", getForwardDistance());

            robot.telemetry.update();
        }
        stopPower();
    }

    void updateDashboardField() {
        TelemetryPacket packet = new TelemetryPacket();
        Canvas field = packet.fieldOverlay();
        FtcDashboard.getInstance().sendTelemetryPacket(packet);
    }

    public void anglesTelemetry() { // Fix
        // telemetry - Angles (XYZ)
        Orientation heading = imu.getAngularOrientation();
//        robotType.telemetry.addData("First angle (most used angle)",heading.firstAngle);
        robot.telemetry.addData("Second angle", heading.secondAngle);
        robot.telemetry.addData("Third angle", heading.thirdAngle);
    }

    public double hoverBoardMode() {
        float angle = imu.getAngularOrientation().secondAngle;
        if (Math.abs(angle) > 1) {
            double power = -angle / 20;
            setPower(power, 0, 0);
            return power;
        } else {
            return 0;
        }
    }

    public boolean zeroOnTargetOnes() {
        final double offset = rodPipline.getRodCenterOffset();
        final double width = rodPipline.getRodWidth();
        final boolean isCone = !rodPipline.isYellow();
        final double targetWidth = isCone ? 360 : 170;
        double f = (targetWidth - width) / targetWidth;
        double s = -Math.signum(f);
        f = Math.abs(f) / 2.5;
        f = Math.min(f, 0.2);
        double t = offset / 2;

        setPower(f * s, t, 0);
        Log.d("Sci", String.format("zeroOnTarget w:  %3.1f/%3.1f, f: %1.2f, turn %1.3f", width, targetWidth, f * s, offset));
        if ((Math.abs(t) < 0.05) && (f < 0.05)) {
            Log.d("Sci", String.format("zeroOnTarget break on: w: %3.1f, f: %1.2f, turn %1.3f", width, f * s, offset));
            return true;
        }
        return false;
    }

//    public void update_dashboard_field() {
////        final double field_width = 5.75 * tile;
//        double x_ = this.getPosX() * meters_to_inches;
//        double y_ = this.getPosY() * meters_to_inches;
//
//        double lastx = robot_pathx.get(robot_pathx.size() - 1);
//        double lasty = robot_pathy.get(robot_pathy.size() - 1);
//        if (Math.abs(lastx - x_) > 1 || Math.abs(lasty - y_) > 1) {
//            robot_pathx.add(x_);
//            robot_pathy.add(y_);
//
//            TelemetryPacket packet = new TelemetryPacket();
//            Canvas canvas = packet.fieldOverlay();
//
//            canvas.setStroke("tomato");
//            canvas.strokePolyline(to_d_katan(robot_pathx), to_d_katan(robot_pathy));
//
//            FtcDashboard.getInstance().sendTelemetryPacket(packet);
//        }
//    }
//
//    private double[] to_d_katan(ArrayList<Double> arr) {
//        double[] a = new double[arr.size()];
//        for (int i = 0; i < arr.size(); i++) {
//            a[i] = arr.get(i);
//        }
//
//        return a;
//    }

    public void zeroOnTarget() {
        ElapsedTime timer = new ElapsedTime();
        while (robot.opModeIsActive() && rodPipline.isRodDetected() && (timer.seconds() <= 3)) {
            if (zeroOnTargetOnes()) break;
            robot.sleep(10);
        }
        Log.d("Sci", "zeroOnTarget ==============================================================");
        setPower(0, 0, 0);
    }

    public enum ROBOT {
        SCORPION,
        COBALT,
        JACCOUSE,
        CONSTANTIN,
        GLADOS
    }

    // DriveMode - (direction & origin of IMU_Integrator)
    public enum DriveMode {
        LEFT(1, new Point(0, 0), new Point(-1, -1)),
        RIGHT(2, new Point(0, 0), new Point(1, 1));

        //        public static double tile = 0.6;
        public int index;
        public Point origin, direciton;

        DriveMode(int index, Point origin, Point direction) {
            this.index = index;
            this.origin = origin;
            this.direciton = direction;
        }
    }

    public enum Axis {
        x, y
    }

    public static class GotoSettings {
        public double power, tolerance, timeout;
        public boolean noSlowdown;

        public GotoSettings(double power, double tolerance, double timeout, boolean noSlowdown) {
            this.power = power;
            this.tolerance = tolerance;
            this.timeout = timeout;
            this.noSlowdown = noSlowdown;
        }

        public static class Builder {
            double power, tolerance, timeout;
            boolean noSlowdown;

            public Builder setPower(double power) {
                this.power = power;
                return this;
            }

            public Builder setTolerance(double tolerance) {
                this.tolerance = tolerance;
                return this;
            }

            public Builder setTimeout(double timeout) {
                this.timeout = timeout;
                return this;
            }

            public Builder setSlowdownMode(boolean active) {
                this.noSlowdown = active;
                return this;
            }

            public GotoSettings build() {
                try {
                    return new GotoSettings(power, tolerance, timeout, noSlowdown);
                } catch (Exception e) {
                    e.printStackTrace();
                    return null;
                }
            }
        }
    }
}