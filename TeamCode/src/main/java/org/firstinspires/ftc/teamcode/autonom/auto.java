package org.firstinspires.ftc.teamcode.autonom;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.drive.DriveConstants;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.main.mecanisme;
import org.firstinspires.ftc.teamcode.reconoastere.Detection;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;
import org.opencv.core.Mat;

@Autonomous(group = "auto")
public class auto extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {

        Pose2d start = new Pose2d(0,0,Math.toRadians(0));
        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);
        mecanisme mecanisme = new mecanisme(hardwareMap);

        drive.setPoseEstimate(start);
        mecanisme.pivot.setPosition(mecanisme.Pivot_SusDeTot);
        mecanisme.turn.setPosition(mecanisme.Turn_FRONT);


        TrajectorySequence ajungere = drive.trajectorySequenceBuilder(start)
                .splineToLinearHeading(new Pose2d(50, -2.5, Math.toRadians(0)), Math.toRadians(0))
                .addDisplacementMarker(2 ,()->{
                    mecanisme.slidePosition(1240,1);

                })
                .turn(Math.toRadians(-47))
                .build();

        TrajectorySequence stack = drive.trajectorySequenceBuilder(ajungere.end())
                .turn(Math.toRadians(137))
                .lineToLinearHeading(new Pose2d(50, 19, Math.toRadians(90)))
                .addDisplacementMarker(2 ,()->{
                    mecanisme.slidePosition(250,1);
                    mecanisme.pivot.setPosition(mecanisme.Pivot_DOWN);

                })
                .build();

        TrajectorySequence pole = drive.trajectorySequenceBuilder(stack.end())
                .lineToLinearHeading(new Pose2d(50, -16, Math.toRadians(90)))
                .addDisplacementMarker(2 ,()->{
                    mecanisme.slidePosition(1230,1);
                    mecanisme.turn.setPosition(0.59);

                })
                .build();


        waitForStart();

        drive.followTrajectorySequence(ajungere);
        mecanisme.pivot.setPosition(mecanisme.Pivot_DOWN);
        sleep(300);
        mecanisme.grip.setPosition(mecanisme.Gripper_OPEN);
        mecanisme.slidePosition(0,1);
        mecanisme.pivot.setPosition(mecanisme.Pivot_SusDeTot);
        sleep(1000);


        drive.followTrajectorySequence(stack);
        mecanisme.grip.setPosition(mecanisme.Gripper_CLOSE);
        sleep(300);
        mecanisme.slidePosition(400,1);

        mecanisme.pivot.setPosition(mecanisme.Pivot_UP);
        drive.followTrajectorySequence(pole);
        mecanisme.pivot.setPosition(mecanisme.Pivot_DOWN);

        mecanisme.grip.setPosition(mecanisme.Gripper_OPEN);


        sleep(2000);



    }

    public void staking (int position, SampleMecanumDrive drive, TrajectorySequence t, mecanisme mecanisme){
        TrajectorySequence stack = drive.trajectorySequenceBuilder(t.end())
                .addDisplacementMarker(()->{
                    mecanisme.grip.setPosition(mecanisme.Gripper_OPEN);
                    mecanisme.pivot.setPosition(mecanisme.Pivot_DOWN);
                    sleep(200);
                    mecanisme.pivot.setPosition(mecanisme.Pivot_UP);
                })
                .addDisplacementMarker(2, ()->{
                    mecanisme.pivot.setPosition(mecanisme.Pivot_DOWN);
                    mecanisme.turn.setPosition(mecanisme.Turn_FRONT);
                    sleep(500);
                })
                .addDisplacementMarker(4, ()->{
                    mecanisme.slidePosition(position,1);
                })
                .lineToConstantHeading(new Vector2d(-19.5,-51.5))
                .build();

        //drive.followTrajectorySequence(stack);
    }
}
