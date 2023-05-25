package org.firstinspires.ftc.teamcode.main;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;

@TeleOp(name = "tlp")
public class tlp extends LinearOpMode {

    enum FieldState{
        COLLECT,
        SCORE
    };

    private FieldState state;

    @Override
    public void runOpMode() throws InterruptedException {

        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);
        mecanisme mecanisme = new mecanisme(hardwareMap);

        state = FieldState.COLLECT;

        double Gripper_OPEN = 1;
        double Gripper_CLOSE = 0.5;
        double Pivot_UP = 0.2;
        double Pivot_DOWN = 0.3;
        double Swing_FRONT = 0.5;
        double Swing_RIGHT = 0.4;
        double Swing_LEFT = 0.6;

        int pos = 0;
        int LOW  = -410;
        int MID = -820;
        int HIGH = -1155;

        waitForStart();

        while(opModeIsActive() && !isStopRequested()){

            drive.setWeightedDrivePower(new Pose2d (
                    -gamepad1.left_stick_y,
                    -gamepad1.left_stick_x,
                    -gamepad1.right_stick_x
            ));

            if(gamepad2.right_bumper){mecanisme.pivot.setPosition(Pivot_UP);}
            if(gamepad2.left_bumper){mecanisme.pivot.setPosition(Pivot_DOWN);}

            mecanisme.slidePosition(pos,1);
            pos+= (gamepad2.right_trigger-gamepad2.left_trigger);

            //CORECTIE
            if (mecanisme.slide.getCurrentPosition()>0){mecanisme.slidePosition(0,1);}
            if(mecanisme.slide.getCurrentPosition()<-1500){mecanisme.slidePosition(-1500,1);}


            switch (state){
                case COLLECT:
                    if(gamepad1.right_bumper){mecanisme.grip.setPosition(Gripper_CLOSE);}
                    if(gamepad1.left_bumper){mecanisme.grip.setPosition(Gripper_OPEN);}

                    //LOW JUNCTION
                    if(gamepad2.cross && mecanisme.grip.getPosition()<0.6){
                        pos = LOW;
                        mecanisme.pivot.setPosition(Pivot_UP);

                        state = FieldState.SCORE;
                    }

                    //MID JUNCTION
                    if(gamepad2.circle & mecanisme.grip.getPosition()<0.6){
                        pos = MID;
                        mecanisme.pivot.setPosition(Pivot_UP);

                        state = FieldState.SCORE;
                    }

                    //HIGH JUNCTION
                    if(gamepad2.triangle &&  mecanisme.grip.getPosition()<0.6){
                        pos = HIGH;
                        mecanisme.pivot.setPosition(Pivot_UP);

                        state = FieldState.SCORE;
                    }

                    break;

                case SCORE:
                    if(gamepad2.dpad_up){mecanisme.swing.setPosition(Swing_FRONT);}
                    if(gamepad2.dpad_right){mecanisme.swing.setPosition(Swing_LEFT);}
                    if(gamepad2.dpad_left){mecanisme.swing.setPosition(Swing_RIGHT);}

                    if(gamepad2.cross){pos = LOW;}
                    if(gamepad2.circle){pos = MID;}
                    if(gamepad2.triangle){pos = HIGH;}

                    if(gamepad1.left_bumper){
                        mecanisme.grip.setPosition(Gripper_OPEN);
                        sleep(1000);
                        pos = 0;

                        state = FieldState.COLLECT;
                    }

                    break;
            }

            telemetry.addData("STATE", state);
            telemetry.addData("GRIPPER", mecanisme.grip.getPosition());
            telemetry.addData("PIVOT", mecanisme.pivot.getPosition());
            telemetry.addData("SWING", mecanisme.swing.getPosition());
            telemetry.addData("SLIDE", mecanisme.slide.getCurrentPosition());
            telemetry.update();
        }

    }
}
