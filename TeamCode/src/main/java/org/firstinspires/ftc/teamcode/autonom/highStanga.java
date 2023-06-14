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

@Autonomous(group = "auto")
public class highStanga extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        Pose2d start = new Pose2d(0,0,Math.toRadians(90));
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

        //TODO preload si alinierea cu stiva

        TrajectorySequence preload = drive.trajectorySequenceBuilder(start)
                .lineTo(
                        new Vector2d(0, 37),
                        SampleMecanumDrive.getVelocityConstraint(slowerVelocity, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                        SampleMecanumDrive.getAccelerationConstraint(slowerAcceleration)
                )
                .addDisplacementMarker(2, ()->{
                    mecanisme.pivot.setPosition(mecanisme.Pivot_UP);
                    mecanisme.slidePosition(920, 1);
                    mecanisme.turn.setPosition(mecanisme.Turn_LEFT);
                })
                .build();


        TrajectorySequence align = drive.trajectorySequenceBuilder(preload.end())
                .addDisplacementMarker(()->{
                    mecanisme.pivot.setPosition(mecanisme.Pivot_DOWN);
                    sleep(500);
                    mecanisme.grip.setPosition(mecanisme.Gripper_OPEN);
                })
                .lineToLinearHeading(new Pose2d(0,52,Math.toRadians(180)))
                .addDisplacementMarker(1, ()->{
                    mecanisme.turn.setPosition(mecanisme.Turn_FRONT);
                    mecanisme.slidePosition(244, 1);
                })
                .lineToConstantHeading(new Vector2d(-19.6,51.5))
                .build();

        //TODO HIGH

        TrajectorySequence junction = drive.trajectorySequenceBuilder(align.end())
                .addDisplacementMarker(()->{
                    mecanisme.grip.setPosition(mecanisme.Gripper_CLOSE);
                    sleep(400);
                    mecanisme.pivot.setPosition(mecanisme.Pivot_UP);
                    mecanisme.slidePosition(1155,1);
                })
                .lineTo(
                        new Vector2d(9.5, 51.5),
                        SampleMecanumDrive.getVelocityConstraint(slowerVelocity, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                        SampleMecanumDrive.getAccelerationConstraint(slowerAcceleration)
                )
                .addDisplacementMarker(2, ()->{
                    mecanisme.turn.setPosition(mecanisme.Turn_LEFT);
                })
                .build();

        waitForStart();

        detection.detectare(telemetry);

        if(opModeIsActive()){
            drive.followTrajectorySequence(preload);
            drive.followTrajectorySequence(align);
            drive.followTrajectorySequence(junction);

            while (i<=4){
                pos = pos - 63*(i);
                staking(pos, drive, junction, mecanisme);
                drive.followTrajectorySequence(junction);
                i++;
            }
        }
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
                .lineToConstantHeading(new Vector2d(-19.5,51.5))
                .build();

        drive.followTrajectorySequence(stack);
    }
}
