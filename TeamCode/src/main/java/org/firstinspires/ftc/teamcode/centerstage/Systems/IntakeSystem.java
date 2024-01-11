package org.firstinspires.ftc.teamcode.centerstage.Systems;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

public class IntakeSystem {
    Servo intakeServo1, intakeServo2;
    DcMotorEx motor;
    LinearOpMode opMode;
    HardwareMap hw;

    public enum ServoPos {
        Open(1),
        Close(0);
        private final double servoPos;

        ServoPos(double servoPos) {
            this.servoPos = servoPos;
        }
    }

    public enum WheelsState {
        Collect,
        Spit,
        Idle
    }

    private boolean isBusy;
    private WheelsState state;

    public IntakeSystem(LinearOpMode opMode) {
        this.opMode = opMode;
    }


    public boolean IsBusy() {
        return isBusy;
    }


    public void setStateIdle() {
        setState(WheelsState.Idle);

    }

    public void setStateSpit() {
        setState(WheelsState.Spit);
    }

    public void setStateCollect() {
        setState(WheelsState.Collect);

    }

    public void init(HardwareMap hw) {
        this.hw = hw; // Cache the hardware map
        motor = hw.get(DcMotorEx.class, "motor");
        setState(WheelsState.Idle);

        intakeServo1 = hw.get(Servo.class, "intakeServo1");
        intakeServo2 = hw.get(Servo.class, "intakeServo2");
        intakeServo2.setDirection(Servo.Direction.REVERSE);
    }

    void setServoPos(double pos) {
        intakeServo1.setPosition(pos);
        intakeServo2.setPosition(pos);
    }

    void setState(WheelsState state) {
        this.state = state;
    }

    /**
     * Moves the intake wheels
     */
    public void spinMotor() {
        double normalPower = 0.1;
        double spitPower = 0.3;
        if (isBusy)
            return; // Don't move if the system is busy
        if (state == WheelsState.Spit) {
            setServoPos(ServoPos.Close.servoPos);
            motor.setDirection(DcMotorSimple.Direction.FORWARD);
            motor.setPower(spitPower);
        } else if (state == WheelsState.Idle) {
            setServoPos(ServoPos.Close.servoPos);
            motor.setPower(0);
            return;
        } else if (state == WheelsState.Collect) {
            setServoPos(ServoPos.Open.servoPos);
            motor.setDirection(DcMotorSimple.Direction.REVERSE);
            motor.setPower(normalPower);
        }
    }

    public WheelsState getState() {
        return this.state;
    }
}
