package org.firstinspires.ftc.teamcode.centerstage.Systems.Arm.States;

import org.firstinspires.ftc.teamcode.centerstage.Systems.Arm.Arm;
import org.firstinspires.ftc.teamcode.centerstage.util.StateMachine.State;

public class HoldState extends State<Arm> {
    Arm owner;

    @Override
    public void enter(Arm owner) {
        this.owner = owner;
        owner.setManualMode(false, 0.12);
    }

    @Override
    public void execute() {
        owner.setManualMode(false, 0.12);
    }

    @Override
    public void exit() {

    }
}
