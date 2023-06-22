package org.firstinspires.ftc.teamcode.autonom;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.main.mecanisme;
import org.firstinspires.ftc.teamcode.reconoastere.Detection;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;

@Autonomous(group = "auto")
public class highLeft extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {

        Pose2d start = new Pose2d(0,0,0);

        Pose2d stack = new Pose2d(2,51.5, 0 );
        Pose2d junction = new Pose2d(-2,51.5, 0 );

        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);
        mecanisme mecanisme = new mecanisme(hardwareMap);

        drive.setPoseEstimate(start);
        mecanisme.pivot.setPosition(mecanisme.Pivot_SusDeTot);

        Detection detection = new Detection();
        detection.VisionInitialization(hardwareMap, telemetry);

        double slowerVelocity = 35;
        double slowerAcceleration = 30;

        int pos = 244;
        int i=1;

        //TODO PRELOAD


        TrajectorySequence preload = drive.trajectorySequenceBuilder(start)
                .lineToLinearHeading(new Pose2d(50,-5.2,0))
                .lineToLinearHeading(new Pose2d(52.5,-5.2,Math.toRadians(315)))
                .addDisplacementMarker(1, ()->{
                    mecanisme.slidePosition(1230,1);
                    mecanisme.pivot.setPosition(mecanisme.Pivot_UP);
                })
                .build();

        TrajectorySequence align = drive.trajectorySequenceBuilder(preload.end())
                .addDisplacementMarker(()->{
                    mecanisme.grip.setPosition(mecanisme.Gripper_OPEN);
                })
                .lineToLinearHeading(new Pose2d(51.5,7,Math.toRadians(90)))
                .addDisplacementMarker(4, ()->{
                    mecanisme.slidePosition(0,0.7);
                    mecanisme.pivot.setPosition(mecanisme.Pivot_DOWN);
                })
                .build();

        waitForStart();

        if(opModeIsActive()){
            drive.followTrajectorySequence(preload);
            drive.followTrajectorySequence(align);
        }
    }
}
