package org.firstinspires.ftc.teamcode.centerstage.Systems;

import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

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
    boolean pixelHere;
    private boolean justStopped;
    private boolean isBusy;

    public IntakeSystem() {
    }

    public boolean IsBusy() {
        return isBusy;
    }

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
        initTime = true;
        this.arm = robot.getComponent(Arm.class);
        motor = hardwareMap.get(DcMotorEx.class, "intakeWheelsMotor");

        farPixelColorSensor = hardwareMap.get(ColorSensor.class, "farPixelColorSensor");
        nearPixelColorSensor = hardwareMap.get(ColorSensor.class, "nearPixelColorSensor");

        intakeServo1 = hardwareMap.get(Servo.class, "intakeServo1");
        intakeServo2 = hardwareMap.get(Servo.class, "intakeServo2");
        intakeServo2.setDirection(Servo.Direction.REVERSE);
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
//        if (state == State.Collect) {
//            boolean farPixelDetected = farPixelColorSensor.red() > 900 || farPixelColorSensor.green() > 900 || farPixelColorSensor.blue() > 900;
//            boolean nearPixelDetected = nearPixelColorSensor.red() > 5000 || nearPixelColorSensor.green() > 5000 || nearPixelColorSensor.blue() > 5000;
//
//            if ((farPixelDetected && nearPixelDetected) && state() == State.Collect) {
//                if (!pixelHere && pixelHereTimer == null) {
//                    pixelHereTimer = new ElapsedTime();
//                } else if (pixelHereTimer != null && pixelHereTimer.seconds() > 0.5) {
//                    pixelHereTimer = null;
//                    pixelHere = true;
//                    stopIntake();
//                }
//            } else {
//                pixelHere = false;
//            }
//        }
    }


    public void setServoPos(double pos) {
        intakeServo1.setPosition(pos);
        intakeServo2.setPosition(pos);
    }

    public void setServoPos(State state) {
        setServoPos(state.servoPos);
    }

    void setState(State state) {
//        if (state == WheelsState.AvoidArm) return;
        prevState = state;
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
        double normalPower = 0.7;
        double spitPower = 0.75;
        if (isBusy) return; // Don't move if the system is busy
//        if (arm != null && arm.pos() < 800 && arm.pos() > 50 && state != WheelsState.Collect && !initTime) {
//            state = WheelsState.AvoidArm;
//        } else {
//        state = prevState;
        if (timer.seconds() >= 0.4 && justStopped && state() == State.Idle) {
            spit();
        } else if (timer.seconds() >= 2 && state() == State.Spit && justStopped) {
            justStopped = false;
            setState(State.Idle);
        }

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
    }

    public State state() {
        return this.state;
    }

    public enum State {
        Collect(1),
        Spit(0),
        Idle(0),
        AvoidArm(0.5);
        private final double servoPos;

        State(double servoPos) {
            this.servoPos = servoPos;
        }
    }
}
