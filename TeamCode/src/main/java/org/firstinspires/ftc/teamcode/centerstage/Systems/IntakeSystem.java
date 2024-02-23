package org.firstinspires.ftc.teamcode.centerstage.Systems;

import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.centerstage.Systems.Arm.Arm;
import org.firstinspires.ftc.teamcode.centerstage.util.ECSSystem.Component;

public class IntakeSystem extends Component {
    public State state = State.Idle, prevState = State.Idle;
    public boolean initTime = true;
    Servo intakeServo1, intakeServo2;
    DcMotorEx motor;
    ElapsedTime timer;
    Arm arm;
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
        motor = hardwareMap.get(DcMotorEx.class, "intakeWheelsMotor");

        intakeServo1 = hardwareMap.get(Servo.class, "intakeServo1");
        intakeServo2 = hardwareMap.get(Servo.class, "intakeServo2");
        intakeServo2.setDirection(Servo.Direction.REVERSE);
        stopIntake();
        initTime = false;
    }

    @Override
    public void start() {
        this.arm = Arm.instance;
        timer = new ElapsedTime();
    }

    @Override
    public void update() {
        spinMotor();
        if (timer.seconds() >= 0.4 && justStopped && state() == State.Idle) {
            spit();
        } else if (timer.seconds() >= 2 && state() == State.Spit && justStopped) {
            justStopped = false;
            setState(State.Idle);
        }
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
    }

    public void setStateAvoidArm() {
        state = State.AvoidArm;
    }

    /**
     * Moves the intake wheels
     */
    public void spinMotor() {
        double normalPower = 0.7;
        double spitPower = 0.6;
        if (isBusy) return; // Don't move if the system is busy
//        if (arm != null && arm.pos() < 800 && arm.pos() > 50 && state != WheelsState.Collect && !initTime) {
//            state = WheelsState.AvoidArm;
//        } else {
//        state = prevState;
        if (state == State.Spit) {
            setServoPos(State.Spit);
            motor.setDirection(DcMotorSimple.Direction.FORWARD);
            motor.setPower(spitPower);
        } else if (state == State.Idle) {
            setServoPos(State.Idle);
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
