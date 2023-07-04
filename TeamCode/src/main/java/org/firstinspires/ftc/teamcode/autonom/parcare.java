package org.firstinspires.ftc.teamcode.autonom;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.acmerobotics.roadrunner.trajectory.TrajectoryBuilder;
import com.acmerobotics.roadrunner.trajectory.TrajectoryGenerator;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.reconoastere.Detection;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;

@Config
@Autonomous(group = "autonomie")
public class parcare extends LinearOpMode {


    @Override
    public void runOpMode() throws InterruptedException {
        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);

        Detection detection = new Detection();
        detection.VisionInitialization(hardwareMap, telemetry);

        Pose2d start = new Pose2d(0, 0, 0);
        drive.setPoseEstimate(start);


        TrajectorySequence CAZ1 = drive.trajectorySequenceBuilder(start)
                .strafeLeft(22)
                .forward(30)
                .build();

        Trajectory CAZ2 = drive.trajectoryBuilder(start)
                .forward(30)
                .build();

        TrajectorySequence CAZ3 = drive.trajectorySequenceBuilder(start)
                .strafeRight(22)
                .forward(30)
                .build();

        waitForStart();

        detection.detectare(telemetry);

        if (opModeIsActive()) {

            if (detection.CAZ == 1) {
                drive.followTrajectorySequence(CAZ1);
            } else if (detection.CAZ == 2) {
                drive.followTrajectory(CAZ2);
            } else {
                drive.followTrajectorySequence(CAZ3);
            }
        }

    }
}
