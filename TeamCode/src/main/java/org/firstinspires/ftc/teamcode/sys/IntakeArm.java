package org.firstinspires.ftc.teamcode.sys;

import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;

public class IntakeArm {
    private Servo Pitch;
    private Servo ClawTurn;
    private Servo ClawRotation;
    private Servo Grip;

    private CRServo Servoleft;
    private CRServo Servoright;

    private DcMotor retriveal = null;



//    public static final double PITCH_UP = 0.79;
//    public static final double CLAW_TURN_UP = 0.49;
    public static final double CLAW_TURN_UP = 0.359;
    public static final double PITCH_UP = 0.765;
    public static final double AUTO_TURN = 0.080;
    public static final double AUTO_TURN2 = 0.034;
    public static final double PITCH_DOWN = 0.36 ;//69767
    public static final double PITCH_DOWN_TELEOP = 0.365 ;
    public static final double AUTO_PITCH_DOWN = 0.565;
    public static final double CLAW_TURN_DOWN = 0.74;
//    private double servo_grip_close_position = 0.05;
//    private double servo_grip_small_open_position = 0.13;
//
//    private double servo_grip_big_open_position = 0.19;
    public static final double GRIP_CLOSE = 0.38;
    public static final double GRIP_OPEN = 1.00;
    public static final double GRIP_OPEN_LIL = 0.11;
    public static final double CLAW_ROTATION_STRAIGHT = 0.36;
    public static final double CLAW_ROTATION_SIDE = 0.03;

    public static final double MID_PITCH = 0.79;
    public static final double TELEOP_MID_PITCH = 0.9;
    public static final  double MID_CLAW_TURN = 0.49;
    public static final  double MID_CLAW_TURN_TELEOP = 0.34;

    public static final double NEW_INTAKE_TRANSFER_AND_INTAKE = 0.152;
    public static final double NEW_INTAKE = 0.120;
    public static final double PITCH_NEW_TRANSFER = 0.76;
    public static final double GO_DOWN_NEW_INTAKE = 0.65;

    public static final double PITCH_INTAKE_SUB = 0.689;
    public static final double PLANET_INTAKE_SUB = 0.54;


    public static final double TICKS_PER_INCHE_RETRIVEAL = 100.66;


    private double autoLookPitch = 0.565;
    private double autoLookClawTurn = 0.915;


    private enum servoState{
        OPEN (0.96),
        CLOSE (1.0);

        Double value;

        private servoState(Double value) {
            this.value = value;
        }
    }
    public IntakeArm(Servo pitch, Servo clawTurn,Servo clawRotation, Servo grip,DcMotor retriveal)
    {
        this.Pitch = pitch;
        this.ClawTurn = clawTurn;
//        this.Servoleft = servoleft;
//        this.Servoright = servoright;
        this.Grip = grip;
        this.ClawRotation = clawRotation;
        this.retriveal = retriveal;

        // Set initial positions
        goUp();
    }

    public void getFromSub()
    {
        this.Pitch.setPosition(PITCH_INTAKE_SUB);//PITCH_DOWN
        this.ClawTurn.setPosition(PLANET_INTAKE_SUB);//CLAW_TURN_DOWN
//        suckIn();
    }

//    public void suckIn()
//    {
//        this.Servoright.setPower(0.8);
//        this.Servoleft.setPower(0.8);
////    }
//    public void suckOut()
//    {
//        this.Servoright.setPower(-0.8);
//        this.Servoleft.setPower(-0.8);
//    }
//    public void suckStop()
//    {
//        this.Servoright.setPower(0);
//        this.Servoleft.setPower(0);
//    }
    public void goDown()
    {
        this.Pitch.setPosition(PITCH_DOWN);//PITCH_DOWN
        this.ClawTurn.setPosition(CLAW_TURN_DOWN);
       //CLAW_TURN_DOWN
//        suckIn();

    }
    public void goDownTeleop()
    {
        this.Pitch.setPosition(PITCH_DOWN_TELEOP);//PITCH_DOWN
        this.ClawTurn.setPosition(CLAW_TURN_DOWN);
        //CLAW_TURN_DOWN
//        suckIn();

    }
    public void camera_goDown()
    {
        this.Pitch.setPosition(autoLookPitch);
        this.ClawTurn.setPosition(autoLookClawTurn);
    }
    public void goDownAuto()
    {
        this.Pitch.setPosition(AUTO_PITCH_DOWN);//PITCH_DOWN
        this.ClawTurn.setPosition(CLAW_TURN_DOWN);
        //CLAW_TURN_DOWN
//        suckIn();

    }

    public void goUp()
    {
        this.Pitch.setPosition(PITCH_UP);//PITCH_UP
        this.ClawTurn.setPosition(CLAW_TURN_UP);//CLAW_TURN_UP
        setClawSide();
//        suckOut();
    }

    public void setClawStraight()
    {
        this.ClawRotation.setPosition(CLAW_ROTATION_STRAIGHT);
    }


    public void setClawSide()
    {
        this.ClawRotation.setPosition(CLAW_ROTATION_SIDE);
    }


    public void closeGrip()
    {
        this.Grip.setPosition(GRIP_CLOSE);
//        suckStop();
    }

    public void openGrip()
    {
        this.Grip.setPosition(GRIP_OPEN);
//        this.suckIn();
    }
//    public void LIL_GRIP_OPEN()
//    {
//        this.Grip.setPosition(GRIP_OPEN_LIL);
//    }

    public void INTAKE_MID()
    {
        Pitch.setPosition(MID_PITCH);
        ClawTurn.setPosition(MID_CLAW_TURN);
    }
    public void TELEOP_INTAKE_MID()
    {
        Pitch.setPosition(PITCH_DOWN);
        ClawTurn.setPosition(CLAW_TURN_UP);
    }

    public void TURN_AUTO_1()
    {
        this.ClawRotation.setPosition(AUTO_TURN);
    }
    public void TURN_AUTO_2()
    {
        this.ClawRotation.setPosition(AUTO_TURN2);
    }
    public void retrievalBack()
    {
        setMotorToPos(retriveal, -85, 1);
    }
    public void retrievalFront()
    {
        setMotorToPos(retriveal, -600, 1);
    }
    public void retrievalFrontLIL()
    {
        setMotorToPos(retriveal, -175, 1);
    }public void retrievalFrontLIL2()
    {
        setMotorToPos(retriveal, -225, 1);
    }

    public void retSub()
    {
        setMotorToPos(retriveal, -450, 1);
    }
    public void retrievalCALC()
    {
        setMotorToPos(retriveal, -300, 1);
    }
    public void retrievalFrontAuto2()
    {
        setMotorToPos(retriveal, -550, 1);
    }
    public void retrievalFrontAuto3()
    {
        setMotorToPos(retriveal, -890, 1);
    }
    public void moveRetrievalByInch(double inch) { setMotorToPos(retriveal, (int) Math.floor(-1*inch * TICKS_PER_INCHE_RETRIVEAL), 0.8); }
    public void stopRetrieval()
    {
        retriveal.setPower(0);
        retriveal.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    }

    public void retrievalFrontAuto1()
    {
        setMotorToPos(retriveal, -820, 1);
    }
    public void retrievalGetSpecFromSub()
    {
        setMotorToPos(retriveal, -350, 1);
    }
    public void retrievalGoBack()
    {
        setMotorToPos(retriveal, -4, 1);
    }
    private boolean setMotorToPos(DcMotor motor, int pos, double power)
    {
        motor.setTargetPosition(pos);
        motor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        motor.setPower(power);
        return true;
    }
}
