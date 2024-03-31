package org.firstinspires.ftc.teamcode.centerstage.Systems.Arm.States;

import org.firstinspires.ftc.teamcode.centerstage.Systems.Arm.Arm;
import org.firstinspires.ftc.teamcode.centerstage.util.StateMachine.State;

public class ManualState extends State<Arm> {
    Arm owner;

    @Override
    public void enter(Arm owner) {
        this.owner = owner;
        owner.setManualMode(true, 0);
    }

    @Override
    public void execute() {

    }

    @Override
    public void exit() {
        owner.setTargetPositions(owner.pos());
    }
}
