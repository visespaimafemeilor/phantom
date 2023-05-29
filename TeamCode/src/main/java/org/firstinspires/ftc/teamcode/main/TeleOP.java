package org.firstinspires.ftc.teamcode.main;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;

//TODO    :::::::: ::    ::    :::    ::    :: ::::::::::  ::::::::  ::      ::
//TODO    ::    :: ::    ::   :: ::   ::::  ::     ::     ::      :: ::::  ::::
//TODO    :::::::: ::::::::  :::::::  :: :: ::     ::     ::      :: ::  ::  ::
//TODO    ::       ::    ::  ::   ::  ::  ::::     ::     ::      :: ::      ::
//TODO    ::       ::    :: ::     :: ::    ::     ::      ::::::::  ::      ::

@TeleOp(name = "TeleOP")
public class TeleOP extends LinearOpMode {

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

        int pos = 0;
        int LOW  = 410;
        int MID =920;
        int HIGH = 1155;

        waitForStart();

        while(opModeIsActive() && !isStopRequested()){


            pos+= (gamepad2.right_trigger-gamepad2.left_trigger);

            switch (state){
                case COLLECT:

                    mecanisme.gripper(gamepad1.right_bumper);
                    //LOW JUNCTION
                    if(gamepad2.cross){
                        pos = LOW;
                        mecanisme.pivot.setPosition(mecanisme.Pivot_UP);
                        state = FieldState.SCORE;
                    }

                    //MID JUNCTION
                    if(gamepad2.circle & mecanisme.grip.getPosition()<0.6){
                        pos = MID;
                        mecanisme.pivot.setPosition(mecanisme.Pivot_UP);
                        state = FieldState.SCORE;
                    }

                    //HIGH JUNCTION
                    if(gamepad2.triangle &&  mecanisme.grip.getPosition()<0.6){
                        pos = HIGH;
                        mecanisme.pivot.setPosition(mecanisme.Pivot_UP);
                        state = FieldState.SCORE;
                    }

                    break;

                case SCORE:
                    if(gamepad2.cross){pos = LOW;}
                    if(gamepad2.circle){pos = MID;}
                    if(gamepad2.triangle){pos = HIGH;}

                    if(gamepad2.right_bumper){mecanisme.turn.setPosition(mecanisme.Turn_RIGHT);}
                    if(gamepad2.left_bumper){mecanisme.turn.setPosition(mecanisme.Turn_LEFT);}
                    if(!gamepad2.right_bumper && !gamepad2.left_bumper){mecanisme.turn.setPosition(mecanisme.Turn_FRONT);}

                    if(gamepad1.left_trigger>0.1){mecanisme.pivot.setPosition(mecanisme.Pivot_DOWN);}
                    else {mecanisme.pivot.setPosition(mecanisme.Pivot_UP);}

                    if(gamepad1.right_bumper){
                        if(mecanisme.pivot.getPosition() == mecanisme.Pivot_DOWN){
                            mecanisme.grip.setPosition(mecanisme.Gripper_OPEN);
                        }
                    }

                    if(gamepad2.left_trigger >= 0.5){
                        mecanisme.turn.setPosition(mecanisme.Turn_FRONT);
                        mecanisme.pivot.setPosition(mecanisme.Pivot_DOWN);
                        pos = 0;
                        state = FieldState.COLLECT;
                    }

                    break;
            }

            drive.setWeightedDrivePower(new Pose2d (
                    -gamepad1.left_stick_y,
                    -gamepad1.left_stick_x,
                    -gamepad1.right_stick_x
            ));

            if(!gamepad1.dpad_down){
                if(gamepad1.right_trigger != 0){mecanisme.hopa.setPosition(0);}
                else{mecanisme.hopa.setPosition(0.3);}
            }
            else{mecanisme.hopa.setPosition(0.7);}

            mecanisme.slidePosition(pos,1);

            telemetry.addData("STATE", state);
            telemetry.addData("GRIPPER", mecanisme.grip.getPosition());
            telemetry.addData("PIVOT", mecanisme.pivot.getPosition());
            telemetry.addData("SWING", mecanisme.turn.getPosition());
            telemetry.addData("SLIDE", mecanisme.slide.getCurrentPosition());
            telemetry.update();
        }

    }
}
