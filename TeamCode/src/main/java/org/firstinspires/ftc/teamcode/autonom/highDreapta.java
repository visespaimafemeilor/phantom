package org.firstinspires.ftc.teamcode.autonom;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.acmerobotics.roadrunner.trajectory.TrajectoryBuilder;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.drive.DriveConstants;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.main.mecanisme;
import org.firstinspires.ftc.teamcode.reconoastere.Detection;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;
import org.opencv.core.Mat;

@Autonomous(group = "auto")
public class highDreapta extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {

        Pose2d start = new Pose2d(0, 0, Math.toRadians(0));

        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);
        mecanisme mecanisme = new mecanisme(hardwareMap);

        Detection detection = new Detection();
        detection.VisionInitialization(hardwareMap,telemetry);

        drive.setPoseEstimate(start);
        mecanisme.pivot.setPosition(mecanisme.Pivot_SusDeTot);
        mecanisme.turn.setPosition(mecanisme.Turn_FRONT);


        TrajectorySequence preload = drive.trajectorySequenceBuilder(start)
                .lineToLinearHeading(new Pose2d(51, 2, Math.toRadians(0)))
                .addDisplacementMarker(2, () -> {
                    mecanisme.slidePosition(1240, 1);

                })
                .turn(Math.toRadians(39))
                .build();


        TrajectorySequence align = drive.trajectorySequenceBuilder(preload.end())
               // .turn(Math.toRadians(-129))
                .lineToLinearHeading(new Pose2d(51.5, -22, Math.toRadians(270)))
                .addDisplacementMarker(2, () -> {
//                    mecanisme.slidePosition(220, 1);
                    mecanisme.pivot.setPosition(mecanisme.Pivot_DOWN);
//                    mecanisme.turn.setPosition(mecanisme.Turn_FRONT);

                })
                .build();

        TrajectorySequence pole = drive.trajectorySequenceBuilder(align.end())
                .addDisplacementMarker(()->{
                    mecanisme.grip.setPosition(mecanisme.Gripper_CLOSE);
                    sleep(200);
                    mecanisme.pivot.setPosition(mecanisme.Pivot_UP);
                })
                .lineToLinearHeading(new Pose2d(50.6, 12.6, Math.toRadians(270)))
                .addDisplacementMarker(2, () -> {
                    mecanisme.slidePosition(1250, 1);
                    mecanisme.turn.setPosition(0.41);

                })
                .build();

        TrajectorySequence stack = drive.trajectorySequenceBuilder(pole.end())
                .addDisplacementMarker(()->{
                    mecanisme.pivot.setPosition(mecanisme.Pivot_UP);
                    mecanisme.turn.setPosition(mecanisme.Turn_FRONT);
                })
                .lineToLinearHeading(new Pose2d(51.4, -22, Math.toRadians(270)))
                .addDisplacementMarker(2, () -> {
                    mecanisme.pivot.setPosition(mecanisme.Pivot_DOWN);
//                    mecanisme.turn.setPosition(mecanisme.Turn_FRONT);

                })
                .build();

        Trajectory CAZ1 = drive.trajectoryBuilder(pole.end())
                .lineToLinearHeading(new Pose2d(50, -21, Math.toRadians(270)))
                .build();

        Trajectory CAZ2 = drive.trajectoryBuilder(pole.end())
                .lineToLinearHeading(new Pose2d(51.5, 0, Math.toRadians(270)))
                .build();

        Trajectory CAZ3 = drive.trajectoryBuilder(pole.end())
                .lineToLinearHeading(new Pose2d(51.5, 25, Math.toRadians(270)))
                .build();


        waitForStart();

        detection.detectare(telemetry);

        //todo PPRELOAD
        drive.followTrajectorySequence(preload);
        mecanisme.pivot.setPosition(mecanisme.Pivot_DOWN);
        sleep(250);
        mecanisme.grip.setPosition(mecanisme.Gripper_OPEN);
        mecanisme.slidePosition(0, 0.7);
        mecanisme.pivot.setPosition(mecanisme.Pivot_SusDeTot);

        //todo FIRST
        mecanisme.slidePosition(260, 1);
        drive.followTrajectorySequence(align);
        drive.followTrajectorySequence(pole);
        mecanisme.pivot.setPosition(mecanisme.Pivot_DOWN);
        sleep(100);
        mecanisme.grip.setPosition(mecanisme.Gripper_OPEN);

        //todo SECOND
        mecanisme.slidePosition(210, 1);
        drive.followTrajectorySequence(stack);
        drive.followTrajectorySequence(pole);
        mecanisme.pivot.setPosition(mecanisme.Pivot_DOWN);
        sleep(100);
        mecanisme.grip.setPosition(mecanisme.Gripper_OPEN);

        //todo THIRD
        mecanisme.slidePosition(170, 1);
        drive.followTrajectorySequence(stack);
        drive.followTrajectorySequence(pole);
        mecanisme.pivot.setPosition(mecanisme.Pivot_DOWN);
        sleep(100);
        mecanisme.grip.setPosition(mecanisme.Gripper_OPEN);

        //todo FOURTH
        mecanisme.slidePosition(110, 1);
        drive.followTrajectorySequence(stack);
        drive.followTrajectorySequence(pole);
        mecanisme.pivot.setPosition(mecanisme.Pivot_DOWN);
        sleep(100);
        mecanisme.grip.setPosition(mecanisme.Gripper_OPEN);

        //todo FIFTH
        mecanisme.slidePosition(0, 1);
        drive.followTrajectorySequence(stack);
        drive.followTrajectorySequence(pole);
        mecanisme.pivot.setPosition(mecanisme.Pivot_DOWN);
        sleep(100);
        mecanisme.grip.setPosition(mecanisme.Gripper_OPEN);


        mecanisme.turn.setPosition(mecanisme.Turn_FRONT);
        mecanisme.pivot.setPosition(mecanisme.Pivot_DOWN);
        mecanisme.slidePosition(0, 0.5);

        drive.followTrajectory(CAZ3);

//        if (detection.CAZ == 1) {
//            drive.followTrajectory(CAZ1);
//        } else if (detection.CAZ == 2) {
//            drive.followTrajectory(CAZ2);
//        } else {
//            drive.followTrajectory(CAZ3);
//        }
    }

}
