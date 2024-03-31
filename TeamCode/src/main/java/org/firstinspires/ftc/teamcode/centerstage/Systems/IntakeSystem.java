package org.firstinspires.ftc.teamcode.centerstage.Systems;

import com.qualcomm.hardware.rev.RevBlinkinLedDriver;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.LED;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.centerstage.Autonomous.AutoFlow;
import org.firstinspires.ftc.teamcode.centerstage.Systems.Arm.Arm;
import org.firstinspires.ftc.teamcode.centerstage.util.ECSSystem.Component;
import org.firstinspires.ftc.teamcode.centerstage.util.ECSSystem.ThreadedComponent;

@ThreadedComponent
public class IntakeSystem extends Component {
    public State state = State.Idle, prevState = State.Idle;
    public boolean initTime = true;
    Servo intakeServo1, intakeServo2;
    DcMotorEx motor;
    ElapsedTime timer;
    ElapsedTime pixelHereTimer;
    Arm arm;
    ColorSensor farPixelColorSensor;
    ColorSensor nearPixelColorSensor;
    DistanceSensor nearPixelDistanceSensor;
    boolean pixelsInside;
    RevBlinkinLedDriver blinkinDriver;
    private boolean justStopped;
    private boolean isBusy;

    public void stopIntake() {
        if (state() == State.Collect) {
            justStopped = true;
            timer.reset();
            arm.openClaw(true);
        }
        setState(State.Idle);
    }

    public void spit() {
        setState(State.Spit);
    }

    public void collect() {
        setState(State.Collect);
    }

    @Override
    public void init() {
        if (robot.alliance == AutoFlow.Alliance.RED) {
            LEDPatterns.None.pattern = RevBlinkinLedDriver.BlinkinPattern.RED;
        } else {
            LEDPatterns.None.pattern = RevBlinkinLedDriver.BlinkinPattern.BLUE;
        }
        initTime = true;
        this.arm = robot.getComponent(Arm.class);
        motor = hardwareMap.get(DcMotorEx.class, "intakeWheelsMotor");

        farPixelColorSensor = hardwareMap.get(ColorSensor.class, "farPixelColorSensor");
        nearPixelColorSensor = hardwareMap.get(ColorSensor.class, "nearPixelColorSensor");
        nearPixelDistanceSensor = hardwareMap.get(DistanceSensor.class, "nearPixelColorSensor");

        intakeServo1 = hardwareMap.get(Servo.class, "intakeServo1");
        intakeServo2 = hardwareMap.get(Servo.class, "intakeServo2");
        intakeServo2.setDirection(Servo.Direction.REVERSE);

//        blinkinDriver = hardwareMap.get(RevBlinkinLedDriver.class, "blinkin");

//        setLEDPattern(LEDPatterns.None);

        timer = new ElapsedTime();
        stopIntake();
        initTime = false;
    }

    @Override
    public void start() {
        pixelHereTimer = new ElapsedTime();
    }

    @Override
    public void update() {
        spinMotor();
        if (state == State.Collect) {
            int farRed = farPixelColorSensor.red();
            int farGreen = farPixelColorSensor.green();
            int farBlue = farPixelColorSensor.blue();
            boolean farPixelDetected = (farRed > 500) && (farGreen > 500) && (farBlue > 500);

            int nearRed = nearPixelColorSensor.red();
            int nearGreen = nearPixelColorSensor.green();
            int nearBlue = nearPixelColorSensor.blue();
            boolean nearPixelDetected = (nearRed > 4000) || (nearGreen > 4000) || (nearBlue > 4000);

            double nearDistanceCM = nearPixelDistanceSensor.getDistance(DistanceUnit.CM);

            if (farPixelDetected && nearPixelDetected && (nearDistanceCM < 0.7)) {
                if (!pixelsInside && pixelHereTimer == null) {
                    pixelHereTimer = new ElapsedTime();
                } else if (pixelHereTimer != null && pixelHereTimer.seconds() > 0.75) {
                    pixelHereTimer = null;
                    pixelsInside = true;
                    stopIntake();
//                    LEDPatterns patternFar = getPatternByDistanceSensor(farRed, farGreen, farBlue);
//                    setLEDPattern(patternFar);
                }
            } else {
                pixelsInside = false;
            }
        }
        prevState = state;
    }


    public void setServoPos(double pos) {
        intakeServo1.setPosition(pos);
        intakeServo2.setPosition(pos);
    }

    public LEDPatterns getPatternByDistanceSensor(double r, double g, double b) {
        if (!pixelsInside) return LEDPatterns.None;
        LEDPatterns pattern = LEDPatterns.White;
        // TODO: color by checks with if's
        return pattern;
    }

    public void setLEDPattern(LEDPatterns pattern) {
        blinkinDriver.setPattern(pattern.pattern);
    }

    public void setServoPos(State state) {
        setServoPos(state.servoPos);
    }

    void setState(State state) {
//        if (state == WheelsState.AvoidArm) return;
        this.state = state;
        if (state == State.Idle) {
            arm.openClaw(true);
        }
    }

    public void setStateAvoidArm() {
        state = State.AvoidArm;
    }

    /**
     * Moves the intake wheels
     */
    public void spinMotor() {
        double normalPower = 1;
        double spitPower = 0.75;
        if (isBusy) return; // Don't move if the system is busy
//        if (arm != null && arm.pos() < 800 && arm.pos() > 50 && state != WheelsState.Collect && !initTime) {
//            state = WheelsState.AvoidArm;
//        } else {
//        state = prevState;
        if (timer.seconds() >= 0.5 && justStopped && state() == State.Idle) {
            spit();
        } else if (timer.seconds() >= 1.5 && state() == State.Spit && justStopped) {
            justStopped = false;
            setState(State.Idle);
        }
//        if (prevState != state) {
        if (state == State.Spit) {
            setServoPos(State.Spit);
            motor.setDirection(DcMotorSimple.Direction.FORWARD);
            motor.setPower(spitPower);
        } else if (state == State.Idle) {
//            setServoPos(State.Idle);
            motor.setPower(0);
        } else if (state == State.Collect) {
            setServoPos(State.Collect);
            motor.setDirection(DcMotorSimple.Direction.REVERSE);
            motor.setPower(normalPower);
        }
//        }
    }

    public State state() {
        return this.state;
    }

    public enum State {
        Collect(1), Spit(0), Idle(0), AvoidArm(0.5);
        private final double servoPos;

        State(double servoPos) {
            this.servoPos = servoPos;
        }
    }

    public enum LEDPatterns {
        Green(RevBlinkinLedDriver.BlinkinPattern.GREEN),
        Purple(RevBlinkinLedDriver.BlinkinPattern.DARK_BLUE),
        White(RevBlinkinLedDriver.BlinkinPattern.WHITE),
        None(RevBlinkinLedDriver.BlinkinPattern.BLUE),
        Yellow(RevBlinkinLedDriver.BlinkinPattern.YELLOW);
        public RevBlinkinLedDriver.BlinkinPattern pattern;

        LEDPatterns(RevBlinkinLedDriver.BlinkinPattern p) {
            this.pattern = p;
        }
    }
}
