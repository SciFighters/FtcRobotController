package org.firstinspires.ftc.teamcode.centerstage.Systems.Arm;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.TouchSensor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.centerstage.Autonomous.AutoFlow;
import org.firstinspires.ftc.teamcode.centerstage.GLaDOSBlue;
import org.firstinspires.ftc.teamcode.centerstage.Systems.Arm.States.GoToState;
import org.firstinspires.ftc.teamcode.centerstage.Systems.Arm.States.GraceHoldTimeState;
import org.firstinspires.ftc.teamcode.centerstage.Systems.Arm.States.HoldState;
import org.firstinspires.ftc.teamcode.centerstage.Systems.Arm.States.IdleState;
import org.firstinspires.ftc.teamcode.centerstage.Systems.Arm.States.ManualState;
import org.firstinspires.ftc.teamcode.centerstage.Systems.DriveClass;
import org.firstinspires.ftc.teamcode.centerstage.Systems.RobotControl;
import org.firstinspires.ftc.teamcode.centerstage.util.ECSSystem.Component;
import org.firstinspires.ftc.teamcode.centerstage.util.ECSSystem.Robot;
import org.firstinspires.ftc.teamcode.centerstage.util.ECSSystem.ThreadedComponent;
import org.firstinspires.ftc.teamcode.centerstage.util.StateMachine.State;
import org.firstinspires.ftc.teamcode.centerstage.util.StateMachine.StateMachine;
import org.firstinspires.ftc.teamcode.freight_frenzy.util.MathUtil;


/**
 * The Arm class controls the movement of an arm using two DC motors.
 */
@ThreadedComponent
public class Arm extends Component {
    public static Arm instance;
    public static double holdPower = 0.1;
    private final double deadZone = 0.05;
    private final double graceTimeLimit = 0.25;
    public State<Arm> gotoState, graceHoldTimeState, holdState, idleState, manualState;
    public DcMotorEx lift1, lift2;
    public TouchSensor touchSensor;
    public StateMachine<Arm> stateMachine;
    public boolean isPlacePixelBusy = false;
    DriveClass drive;
    ElapsedTime elapsedTime;
    private int targetPos;
    private ElapsedTime timer;
    // Constants for proportional control in GoToState
    private Servo frontServo, backServo;
    private DistanceSensor distanceSensor;
    private int lastArmPos;
    private double lastTime = 0;

    public Arm() {

    }

    /**
     * Initializes the Arm object.
     */
    @Override
    public void init() {
        elapsedTime = new ElapsedTime();
        initStateMachine();
        this.timer = new ElapsedTime();
        this.lift1 = hardwareMap.get(DcMotorEx.class, "lift1");
        this.touchSensor = hardwareMap.get(TouchSensor.class, "armLimitSensor1");
        this.lift2 = hardwareMap.get(DcMotorEx.class, "lift2");
        this.lift1.setDirection(DcMotorEx.Direction.FORWARD);
        this.lift2.setDirection(DcMotorSimple.Direction.REVERSE);
        this.frontServo = hardwareMap.get(Servo.class, "frontClawServo");
        this.backServo = hardwareMap.get(Servo.class, "backClawServo");
        this.distanceSensor = hardwareMap.get(DistanceSensor.class, "armDistanceSensor");
        initMotors();
        resetClaw();
        resetArm();
        closeClaw(true);
        stateMachine.changeState(idleState);
        instance = this;
    }

    private void initStateMachine() {
        gotoState = new GoToState();
        graceHoldTimeState = new GraceHoldTimeState();
        holdState = new HoldState();
        idleState = new IdleState();
        manualState = new ManualState();
        stateMachine = new StateMachine<>(this);
    }

    @Override
    public void start() {
        drive = robot.getComponent(DriveClass.class);
    }

    private void initMotors() {
        telemetry.addData("Lift Init", "In progress");
        telemetry.update();

        lift1.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        lift2.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);


        telemetry.addData("Lift Init", "Finished");
        telemetry.update();
    }

    public void resetClaw() {
        double minServoPos = 0.35;
        frontServo.scaleRange(minServoPos, 1);
        backServo.scaleRange(minServoPos, 1);
    }

    /**
     * Resets the arm to its initial state. To be implemented.
     */
    public void resetArm() {
        setTargetPositions(0);
        ElapsedTime timer = new ElapsedTime();
        timer.reset();
        lift1.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        lift2.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        if (!touchSensor.isPressed()) {
            while (!touchSensor.isPressed() && (!robot.isStopRequested()) && timer.seconds() < 2) {
                setMotorsPower(-0.3);
                telemetry.addData("BUTTON STATUS: ", touchSensor.isPressed());
                telemetry.update();
            }
        }
        lift1.setPower(0);
        lift2.setPower(0);

        resetArmPos();
    }

    /**
     * Runs the Arm thread continuously, handling state transitions.
     */
    @Override
    public void update() {
        lastArmPos = pos();
        lastTime = elapsedTime.seconds();

        handleStates();
//        if (pos() < 800 && pos() > 60 && intakeSystem.state() != IntakeSystem.WheelsState.Collect && !intakeSystem.initTime) {
//            intakeSystem.setStateAvoidArm();
//        }
//        if ((Math.abs(robot.gamepad2.left_stick_x) > 0.05 || Math.abs(robot.gamepad2.left_stick_y) > 0.05) && holdPower != 0.1) {
//            holdPower = 0.1;
//        }
    }

    /**
     * Checks if the current drawn by the motors exceeds a limit.
     *
     * @return True if the current is over the limit, false otherwise.
     */
    public boolean isOverCurrentLimit() {
        return getCurrent() > 1.5; // Adjust this limit based on your requirements
    }

    /**
     * Moves the arm to the specified position using fixed control.
     *
     * @param pos The target position for the arm.
     */
    public void goToPos(Position pos) {
        goToPos(pos.liftPos);
    }

    /**
     * Moves the arm to the specified position using proportional control.
     *
     * @param pos The target position for the arm.
     */
    public void goToPos(int pos) {
        setTargetPositions(pos);
        stateMachine.changeState(gotoState);
    }

    /**
     * @param position arm height
     */
    public void placePixels(Position position) {
        isPlacePixelBusy = true;
        double distance = Math.min(drive.getDistanceLeftSensorDistance(), drive.getDistanceRightSensorDistance());
        if (distance < position.distanceFromBackdrop && pos() > position.liftPos) {
            alignToBoard(position);
            goToPos(position);
        } else {
            goToPos(position);
            alignToBoard(position);
        }
        isPlacePixelBusy = false;
    }

    public void alignToBoard(Position position) {
        double targetHeading = (robot.alliance == AutoFlow.Alliance.BLUE ? 180 : -180) - (robot.type == Robot.TYPE.Auto ? 90 : 0);
        ElapsedTime timer = new ElapsedTime();
        double distance = Math.min(drive.getDistanceLeftSensorDistance(), drive.getDistanceRightSensorDistance());
        while (!MathUtil.approximately(distance, position.distanceFromBackdrop, 0.5) && timer.seconds() < 3) {
            double gain = 0.023 * (robot.alliance == AutoFlow.Alliance.RED ? -1 : 1);
            double deltaAngle = drive.getDeltaHeading(targetHeading);
            double turn = deltaAngle * gain / 2;
            double delta = position.distanceFromBackdrop - distance;
            if (delta > 100) {
                break;
            }
            if (robot.type == Robot.TYPE.Auto) {
                gain *= -1;
            }
            double power = gain * delta / 2;

            robot.telemetry.addData("Place pixel delta distance", delta);
            robot.telemetry.addData("power", power);
            robot.telemetry.update();
            drive.setPowerOriented(pow(-robot.gamepad1.left_stick_y / 2), power + pow(robot.gamepad1.left_stick_x / 2), pow(robot.gamepad1.right_stick_x / 2), true);
            distance = Math.min(drive.getDistanceLeftSensorDistance(), drive.getDistanceRightSensorDistance());
//            if (Math.abs(robot.gamepad1.left_stick_x) > 0.1 || Math.abs(robot.gamepad1.left_stick_y) > 0.1 || Math.abs(robot.gamepad2.left_stick_x) > 0.1 || Math.abs(robot.gamepad2.left_stick_y) > 0.1) {
//                break;
//            }
        }
        if (robot.type == Robot.TYPE.Auto) drive.setPowerOriented(0, 0, 0, true);
    }

    public double alignToBoardTeleOp(Position position) {
        if (RobotControl.boardAlignSensor == -1) {
            double rightDistance = drive.getDistanceRightSensorDistance();
            double leftDistance = drive.getDistanceLeftSensorDistance();
            RobotControl.boardAlignSensor = rightDistance > leftDistance ? 1 : 2;
        }

        double distance = RobotControl.boardAlignSensor == 1 ?
                drive.getDistanceLeftSensorDistance() : drive.getDistanceRightSensorDistance();
        double deltaDistance = position.distanceFromBackdrop - distance;

        double gain = 0.023;

        return MathUtil.clamp(deltaDistance * gain, -0.4, 0.4);
    }

    public double pow(double x) {
        return Math.pow(x, 2) * Math.signum(x);
    }

    /**
     * Gets the current position of the arm.
     *
     * @return The current position in encoder counts.
     */
    public int pos() {
        return lift1.getCurrentPosition();
    }

    public int pos2() {
        return lift2.getCurrentPosition();
    }

    public double getCurrent2() {
        return lift2.getCurrent(CurrentUnit.AMPS);
    }

    /**
     * Gets the target position for the arm.
     *
     * @return The target position in encoder counts.
     */
    public int targetPos() {
        return this.targetPos;
    }

    /**
     * Sets the target positions for the arm.
     *
     * @param pos The target position in encoder counts.
     */
    public void setTargetPositions(int pos) {
        targetPos = pos;
    }

    /**
     * Sets the manual control mode and power for the arm.
     *
     * @param manual True to enable manual mode, false for position control.
     * @param power  The manual power if in manual mode.
     */
    public void setManualMode(boolean manual, double power) {
        if (manual) {
            lift1.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
            lift2.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
        } else {
            lift1.setTargetPosition(targetPos);
            lift2.setTargetPosition(targetPos);
            lift1.setTargetPositionTolerance(10);
            lift2.setTargetPositionTolerance(10);
            lift1.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);
            lift2.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);
            setMotorsPower(power);
        }
    }

    /**
     * Handles the state machine execution.
     */
    public void handleStates() {
        stateMachine.execute();
        if (targetPos() == 0) {
            if (!touchSensor.isPressed() && pos() < 500 &&
                    (int) timer.seconds() % 2 == 0 &&
                    (lift1.getPower() == 0 || stateMachine.getCurrentState() == holdState)) {
                resetArm();
            } else if (touchSensor.isPressed() && pos() != 0) {
//            resetArm();
                resetArmPos();
            }
        }
    }

    public void resetArmPos() {
        lift1.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        lift2.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);

        lift1.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
        lift2.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
        setTargetPositions(0);
    }

    /**
     * Checks if the motors are currently busy with a task.
     *
     * @return True if the motors are busy, false otherwise.
     */
    public boolean areMotorsBusy() {
        return !lift1.isBusy() && !lift2.isBusy();
    }

    /**
     * Sets the power for the arm motors.
     *
     * @param power The power to set to the motors.
     */
    public void setPower(double power) {
        if (outOfDeadZone(power)) {
            stateMachine.changeState(manualState);
            setMotorsPower(power);
        } else {
            if (stateMachine.getCurrentState() != gotoState) {
                setTargetPositions(pos());
                stateMachine.changeState(holdState);
            }
        }
    }

    public double distanceSensorDistance() {
        return distanceSensor.getDistance(DistanceUnit.CM);
    }

    /**
     * Sets the power for the arm motors.
     *
     * @param power The power to set to the motors.
     */
    public void setMotorsPower(double power) {
        int higherLimit = 65, lowerLimit = 10;
        if (stateMachine.getCurrentState() == gotoState) {
            higherLimit = 100;
        }
        if (!outOfDeadZone(power)) {
            power = 0;
        } else if ((power < 0) && (touchSensor.isPressed())) {
            power = 0;
        } else if (power > 0 && pos() > 2000) {
            if ((stateMachine.getCurrentState() == gotoState && targetPos() >= pos())
                    || (stateMachine.getCurrentState() == manualState)) {
                double distanceSensorDist = distanceSensorDistance();
                if (distanceSensorDist < lowerLimit)
                    power = 0.1;
                else if (distanceSensorDist < higherLimit) {
                    if (power > 0.8) {
                        power = 0.3;
                    }
//                if (stateMachine.getCurrentState() == manualState) {
//                    power = calculateMotorsPower(power, distanceSensorDist);
//
//                }
                }
            }
        }
        if (targetPos() == 0 && !this.touchSensor.isPressed() && pos() < 740 &&
                !robot.opModeInInit() && stateMachine.getCurrentState() == gotoState) {
            power = -0.2;
        }
//    power = calculateMotorsPower(power);
        lift1.setPower(power);
        lift2.setPower(power);
    }

    public void setManualPower(double power) {
        stateMachine.changeState(manualState);
        setMotorsPower(power);
    }

    /**
     * Gets the current drawn by the arm motors.
     *
     * @return The current drawn by the motors in amps.
     */
    public double getCurrent() {
        return lift1.getCurrent(CurrentUnit.AMPS);
    }

    /**
     * Checks if the power is outside the dead zone.
     *
     * @param power The power to be checked.
     * @return True if the power is outside the dead zone, false otherwise.
     */
    public boolean outOfDeadZone(double power) {
        return Math.abs(power) > deadZone;
    }

    /**
     * Sets the position of the claw servos.
     *
     * @param position The position to set the claw servos to.
     */
    public void closeClaw(double position) {
        frontServo.setPosition(position);
        backServo.setPosition(position);
    }

    /**
     * Sets the position of the claw servos based on open or closed state.
     *
     * @param close True to close the claw, false to open it.
     */
    public void closeClaw(boolean close) {
        closeClaw(close ? 1 : 0);
    }

    public double clawPosition() {
        return backServo.getPosition();
    }

    public double getDeadZone() {
        return this.deadZone;
    }

    public void hang() {
        holdPower = 0.15;
        goToPos(Position.Hang);
    }

    public void dropBottomPixel() {
        backServo.setPosition(0);
    }

    public double calculateMotorsPower(double power, double distance) {
        if (power == 0) {
            return 0;
        } else if (pos() < 1000) {
            return power;
        }
        double finalPower = power;
        double kY = GLaDOSBlue.kY;
        double avgMaxVelocity = GLaDOSBlue.avgMaxVelocity;
        double minDist = GLaDOSBlue.minDist;
        double kDistance = avgMaxVelocity / minDist;
        finalPower = Math.min(power, distance * kY) -
                Math.max(0, calculateVelocity() - calculateMaxVelocityToDistance(distance)) * kDistance;
        return finalPower;
    }

    public double calculateVelocity() {
        return (pos() - lastArmPos) / (elapsedTime.seconds() - lastTime);
    }

    public double calculateMaxVelocityToDistance(double distance) {
        double result = distance * GLaDOSBlue.avgMaxVelocityGain;
        RobotControl.multipleTelemetry
                .addData("Arm distance velocity", result / 10);
        return result;
    }

    /**
     * Enumeration for different arm positions.
     */
    public enum Position {
        Home(0, -1), One(4170, 42), Two(3130, 27), Hang(500, -1), Three(2780, 7);
        public final int liftPos;
        public final double distanceFromBackdrop;

        Position(int liftPos, double distanceFromBackdrop) {
            this.liftPos = liftPos;
            this.distanceFromBackdrop = distanceFromBackdrop;
        }
    }
}
