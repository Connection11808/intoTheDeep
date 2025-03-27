package org.firstinspires.ftc.teamcode.state_machine;

public class StateMachineBase {
    protected int state = 0;
    protected boolean IsRunning = false;
    protected int statePrevValue = 0;

    public StateMachineBase() {}

    public void setState(int state) {
        this.IsRunning = true;
        this.state = state;
    }
    public void update(){}
    public void stop(){
        statePrevValue = 0;
        state = 0;
        this.IsRunning = false;
    }

    protected void setPrevValue() {
        statePrevValue = state;
    }

}
