// Drive Class
// Freight Frenzy 2021 - 2022
package org.firstinspires.ftc.teamcode.freight_frenzy.util;

import android.util.Log;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.RobotLog;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.robotcore.external.navigation.Position;
import org.firstinspires.ftc.robotcore.external.navigation.Velocity;

public class DriveClass {
    //region DON'T TOUCH
    final double tile = 0.6;
    private boolean useEncoders = false;
    private boolean useBrake = false;

    public DriveClass useEncoders() {
        this.useEncoders = true;
        return this;
    }

    public DriveClass useBrake() {
        this.useBrake = true;
        return this;
    }

    //endregion DON'T TOUCH

    private LinearOpMode opMode; // First I declared it as OpMode now its LinearOpMode

    volatile private DcMotorEx fl = null;
    volatile private DcMotorEx fr = null;
    volatile private DcMotorEx bl = null;
    volatile private DcMotorEx br = null;

    private BNO055IMU imu = null;
    private BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();

    private boolean fieldOriented = true;
    private double angleOffset = 0;

    private int fl_startPos = 0;
    private int fr_startPos = 0;
    private int bl_startPos = 0;
    private int br_startPos = 0;

    public enum ROBOT {
        SCORPION,
        COBALT,
        JACCOUSE
    }

    private ROBOT robot;

    private Location startingPosition;

    private double forwardTicksPerMeter;
    private double strafeTicksPerMeter;

    ElapsedTime timer = new ElapsedTime();

    public DriveClass(LinearOpMode opMode, ROBOT robot, Location startingPosition) {
        this.opMode = opMode;
        this.robot = robot;
        this.startingPosition = startingPosition;


        if (robot == ROBOT.JACCOUSE) {
            this.forwardTicksPerMeter = 1562.5;
            this.strafeTicksPerMeter = 1645.83;
        } else if (robot == ROBOT.SCORPION) {
            this.forwardTicksPerMeter = 2455;
            this.strafeTicksPerMeter = 2587;
        } else if (robot == ROBOT.COBALT) {
            this.forwardTicksPerMeter = 1753;
            this.strafeTicksPerMeter = 2006;
        }
    }

    public void init(HardwareMap hw) {
        //region get from hw
        fl = hw.get(DcMotorEx.class, "fl");
        fr = hw.get(DcMotorEx.class, "fr");
        bl = hw.get(DcMotorEx.class, "bl");
        br = hw.get(DcMotorEx.class, "br");
        //endregion get from hw

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

        initIMU(hw);
    }

    private void initIMU(HardwareMap hw) {
        imu = hw.get(BNO055IMU.class, "imu 1");

        parameters.angleUnit = BNO055IMU.AngleUnit.DEGREES;
        parameters.accelUnit = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        parameters.accelerationIntegrationAlgorithm = new IMU_Integrator(imu, hw, forwardTicksPerMeter, strafeTicksPerMeter);

        imu.initialize(parameters);

        opMode.telemetry.addData("Gyro", "calibrating...");
        opMode.telemetry.update();

        ElapsedTime timer = new ElapsedTime();
        timer.reset();
        while (!imu.isGyroCalibrated() && !opMode.isStopRequested() && timer.seconds() < 5) {
            opMode.sleep(50);
        }
        if (imu.isGyroCalibrated()) {
            opMode.telemetry.addData("Gyro", "Done Calibrating");
        } else {
            opMode.telemetry.addData("Gyro", "Gyro/IMU Calibration Failed");
        }

        imu.startAccelerationIntegration(new Position(DistanceUnit.METER, this.startingPosition.x, this.startingPosition.y, 0, 0), new Velocity(), 2);

        opMode.telemetry.update();

        RobotLog.d("IMU status: %s", imu.getSystemStatus().toShortString());
        RobotLog.d("IMU calibration status: %s", imu.getCalibrationStatus().toString());
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
        if (fieldOriented != true) {
            setPower(y, turn, x);  // No field oriented
        } else {
            double phiRad = (-getHeading() + angleOffset) / 180 * Math.PI;
            double forward = y * Math.cos(phiRad) - x * Math.sin(phiRad);
            double strafe = y * Math.sin(phiRad) + x * Math.cos(phiRad);
            setPower(forward, turn, strafe);
        }

//		opMode.telemetry.addData("Front","left/right: %d, %d", fl.getCurrentPosition(), fr.getCurrentPosition());
//		opMode.telemetry.addData("Back","left/right: %d, %d", bl.getCurrentPosition(), br.getCurrentPosition());
    }

    public void stopPower() {
        setPower(0, 0, 0);
    }

    public void resetOrientation(double angle) {
        imu.initialize(parameters);
        angleOffset = angle;
    }

    public double getHeading() {
        Orientation orientation = imu.getAngularOrientation();
        return -orientation.firstAngle;
    }

    public double getDeltaHeading(double target) {
        double currentAngle = getHeading();
        double delta = target - currentAngle;

        if (delta < 180) {
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
        opMode.telemetry.addData("fl", fl.getCurrentPosition());
        opMode.telemetry.addData("fr", fr.getCurrentPosition());
        opMode.telemetry.addData("bl", bl.getCurrentPosition());
        opMode.telemetry.addData("br", br.getCurrentPosition());

        opMode.telemetry.addData("average", (fl.getCurrentPosition() + fr.getCurrentPosition() + bl.getCurrentPosition() + br.getCurrentPosition()) / 4);

        opMode.telemetry.addData("forwardDist", getForwardDistance());
        opMode.telemetry.addData("strafeDist", getStrafeDistance());
    }

    public void resetPosition() {
        fl_startPos = fl.getCurrentPosition();
        fr_startPos = fr.getCurrentPosition();
        bl_startPos = bl.getCurrentPosition();
        br_startPos = br.getCurrentPosition();
        opMode.telemetry.addData("RESET POSITION !!!!!!", 0);
    }


    public void turn(double deg, double power) {
        double targetAngle = getHeading() + deg; // zeroAngle
        turnTo(targetAngle, power);
    }

    public void turnTo(double targetAngle, double targetPower) {
        double delta = getDeltaHeading(targetAngle);
        double s = (delta < 0) ? -1 : 1;
        while ((delta * s > 0) && opMode.opModeIsActive()) {

            delta = getDeltaHeading(targetAngle);
            double gain = 0.02;
            double power = gain * delta * targetPower;
            if (Math.abs(power) < 0.1)
                power = 0.1 * Math.signum(power);

            setPower(0, power, 0);

            opMode.telemetry.addData("target", targetAngle);
            opMode.telemetry.addData("current", getHeading());
            opMode.telemetry.addData("delta", delta);
            opMode.telemetry.addData("power", power);
            opMode.telemetry.update();
        }
        stopPower();
    }

    public void goToLocation(Location location, double power, double targetHeading, double tolerance, double timeout) {
        goTo(location.x, location.y, power, targetHeading, tolerance, timeout);
    }
    public void goToLocation(Location location, double power, double tolerance, double timeout) {
        goTo(location.x, location.y, power, location.angle, tolerance, timeout);
    }

    public final long maxIdleCount = 250;
    public void goTo(double x, double y, double targetPower, double targetHeading, double tolerance, double timeout) {
        long goToIdle = 0; //if not moving
        boolean isCheckingIdle = false;
        double lastX = 0;
        double lastY = 0;

        double currentX = getPosX();
        double currentY = getPosY();
        double deltaX = x - currentX;
        double deltaY = y - currentY;
        double startX = currentX;
        double startY = currentY;

        //double totalDist = Math.sqrt(deltaX*deltaX + deltaY*deltaY);
        double totalDist = Math.hypot(deltaX, deltaY);
        double currentDist = 0;

        ElapsedTime timer = new ElapsedTime();

        opMode.telemetry.addData("goto x", x);
        opMode.telemetry.addData("goto y", y);
        opMode.telemetry.addData("goto delta x", deltaX);
        opMode.telemetry.addData("goto delta y", deltaY);
        opMode.telemetry.addData("goto hypocampus total dist", totalDist);
        opMode.telemetry.update();

        while (opMode.opModeIsActive() && (currentDist < totalDist - tolerance)) { // TODO: there a NaN here somewhere (the attack of the NaNs)


            double power = targetPower;

            currentX = getPosX();
            currentY = getPosY();
            deltaX = currentX - startX;
            deltaY = currentY - startY;
            currentDist = Math.hypot(deltaY, deltaX); // distance moved from start position.
            deltaX = x - currentX;
            deltaY = y - currentY;
            double remainDist = Math.hypot(deltaY, deltaX);  // distance left to target.

            //double leftDist = totalDist - currentDist;
            double minPower = 0.2;
            double acclGain = 1.5;
            double acclPower = currentDist * acclGain + minPower;

            double RVy = deltaY / remainDist;  // y velocity ratio
            double RVx = deltaX / remainDist;  // x velocity ratio


            if (acclPower < power) {
                power = acclPower;
            }

            double breakGain = 0.5;
            double breakPower = remainDist * breakGain + minPower;

            if (breakPower < power) { //&& tolerance > 0.05
                power = breakPower;
            }

            double headingErr = getDeltaHeading(targetHeading) / 180;
            double headingGain = 0.7;
            double correction = headingGain * headingErr;
            double Vy = RVy * power;
            double Vx = RVx * power;

            //setPowerOriented(Vy, Vx, correction, true);
            double phiRad = -getHeading() / 180 * Math.PI;
            double forward = Vy * Math.cos(phiRad) - Vx * Math.sin(phiRad);
            double strafe = Vy * Math.sin(phiRad) + Vx * Math.cos(phiRad);
            setPower(forward, correction, strafe);


            opMode.telemetry.addData("distance", "%2.3f, %2.3f", currentDist, remainDist);
            opMode.telemetry.addData("Abs Pos", "X,Y %2.3f, %2.3f", getAbsolutesPosX(), getAbsolutesPosY());
            opMode.telemetry.addData("> currnt", "X,Y: %2.3f, %2.3f", currentX, currentY);
            opMode.telemetry.addData("goto delta", " x,y: %2.3f, %2.3f", deltaX, deltaY);
            opMode.telemetry.addData("goto velos", "f,s: %2.3f, %2.3f", Vy, Vx);
            opMode.telemetry.addData("heading ", getHeading());
            opMode.telemetry.addData("heading error", headingErr);
            opMode.telemetry.addData("power", power);
            opMode.telemetry.update();

            if((timeout != 0 && timeout <= timer.seconds())) break;

            // Idle checker
            double moveRange = 0.001;

            double dx = Math.abs(lastX - currentX);
            double dy = Math.abs(lastY - currentY);
            double dist2 = Math.hypot(dx, dy);
            Log.d("distance from last time (delta)", String.valueOf(dist2));
            if((currentDist / remainDist) < 0.25 && dist2 < moveRange) goToIdle += 1;
            lastX = currentX; lastY = currentY;
//              if(goToIdle >= (long)(maxIdleCount / 2)) break;
        }
        setPower(0, 0, 0);
    }

    private boolean inRange(double min, double max, double value) {
        if(max < min) { double tempMax = max; max = min; min = tempMax; }
        if(value > min && value < max) return true;
        return false;
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

        while (opMode.opModeIsActive() && (RVf != 0) || (RVs != 0)) {

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

            opMode.telemetry.addData("time", timer.milliseconds());
            //position Telemetry:
            opMode.telemetry.addData("x position:", getAbsolutesPosX());
            opMode.telemetry.addData("y position:", getAbsolutesPosY());


            opMode.telemetry.addData("delta forward:", deltaForward);
            opMode.telemetry.addData("speed forward:", Vf);
//			opMode.telemetry.addData("delta strafe:", deltaStrafe);
            opMode.telemetry.addData("speed strafe:", Vs);
            opMode.telemetry.addData("power:", power);
            opMode.telemetry.addData("current pos", getForwardDistance());

            opMode.telemetry.update();
        }
        stopPower();
    }
}
