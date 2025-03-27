package org.firstinspires.ftc.teamcode;

import android.util.Log;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.util.Angle;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;

@Config
@TeleOp(name = "lockToPos",group = "drive")
@Disabled
public class lockToPos extends LinearOpMode {

    SampleMecanumDrive drive;
    String TAG_RR = "rr";
    Pose2d PosToLock;

    Pose2d CLICK_POSITION;
    public static double xyP = 0.2;
    public static double headingP = 0.95;

    @Override
    public void runOpMode() throws InterruptedException {
        drive = new SampleMecanumDrive(hardwareMap);

        Pose2d startPose = new Pose2d(-60, 0, Math.toRadians(90));

        drive.setPoseEstimate(startPose);
        PosToLock = startPose;
        CLICK_POSITION = startPose;
        waitForStart();
        while (opModeIsActive()){
            double axial   = -gamepad1.left_stick_y;  // Note: pushing stick forward gives negative value
            double lateral =  gamepad1.left_stick_x;
            double yaw     =  gamepad1.right_stick_x;
            if ((axial != 0) || (lateral != 0) || (yaw != 0))
            {
                double max;

                double leftFrontPower  = axial + lateral + yaw;
                double rightFrontPower = axial - lateral - yaw;
                double leftBackPower   = axial - lateral + yaw;
                double rightBackPower  = axial + lateral - yaw;
                max = Math.max(Math.abs(leftFrontPower), Math.abs(rightFrontPower));
                max = Math.max(max, Math.abs(leftBackPower));
                max = Math.max(max, Math.abs(rightBackPower));

                if (max > 1.0) {
                    leftFrontPower  /= max;
                    rightFrontPower /= max;
                    leftBackPower   /= max;
                    rightBackPower  /= max;
                }

                drive.setMotorPowers(leftFrontPower, leftBackPower, rightBackPower, rightFrontPower);
                drive.update();
                PosToLock = drive.getPoseEstimate();

            }
            else {
                LockTo(PosToLock);
                drive.update();
            }

        }
    }
    public void LockTo(Pose2d targetPos)
    {
        Pose2d currentPos = drive.getPoseEstimate();
        Pose2d difference = targetPos.minus(currentPos);
        Vector2d xy = difference.vec().rotated(-currentPos.getHeading());

        double heading = Angle.normDelta(targetPos.getHeading() - Angle.normDelta(currentPos.getHeading()));
        Log.d(TAG_RR, "difference" + xy.component1() + " " + xy.component2());
        Log.d(TAG_RR, "heading" + heading);
        if ((Math.abs(xy.component1()) > 0.2) || (Math.abs(xy.component2()) > 0.2) || (Math.abs(heading) > 0.2))
        {
            drive.setWeightedDrivePower(new Pose2d(xy.times(xyP), heading * headingP));

        }
    }
}
