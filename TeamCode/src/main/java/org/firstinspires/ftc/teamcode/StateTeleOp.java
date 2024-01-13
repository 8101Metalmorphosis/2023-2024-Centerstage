package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.FTCutil.ButtonToggle;
import org.firstinspires.ftc.teamcode.FTCutil.ButtonToggleAdvanced;
import org.firstinspires.ftc.teamcode.FTCutil.MathUtil;
import org.firstinspires.ftc.teamcode.Subsystems.RobotBase;


@TeleOp(name = "State TeleOp", group = "!")
public class StateTeleOp extends LinearOpMode {

    RobotBase robot;

    // Button Toggles
    ButtonToggle FOD;
    ButtonToggle alignAprilTag;

    ButtonToggleAdvanced antiTip;


    ButtonToggle intake;
    ButtonToggle transfer;

    // Gamepad 1
    double LY1;
    double LX1;

    double RX1;

    double LEFTTRIGGER1;
    double RIGHTTRIGGER1;

    boolean A1;
    boolean B1;

    boolean Y1;
    boolean BACK1;


    // Gamepad 2
    double LY2;

    double RY2;



    boolean R2BUMPER;


    boolean A2;
    boolean B2;
    boolean Y2;
    boolean X2;


    boolean DPAD_UP2;
    boolean DPAD_LEFT2;
    boolean DPAD_DOWN2;

    public enum States {
        INTAKE,
        TRANSFER,
        PLACE,
        NONE
    }

    @Override
    public void runOpMode() throws InterruptedException {

        robot = new RobotBase(hardwareMap);

        FOD = new ButtonToggle();
        alignAprilTag = new ButtonToggle();

        antiTip = new ButtonToggleAdvanced(ButtonToggleAdvanced.NEUTRAL_STATE);

        intake = new ButtonToggle();
        transfer = new ButtonToggle();


        ElapsedTime doorTime = new ElapsedTime();
        ElapsedTime pivotTime = new ElapsedTime();


        States state = States.NONE;

        boolean firstState = true;
        boolean drop = false;

        while (opModeInInit()) {

        }

        waitForStart();
        if(opModeIsActive()) {
            while(opModeIsActive()) {

                updateInputs();
                FOD.update(A1);
                //alignAprilTag.update();
                //antiTip.update();

                intake.update(Y2);
                transfer.update(X2);


                // Update Robot
                robot.updateDrive(LY1, LX1, RX1, FOD.getState(), false, alignAprilTag.getState(), antiTip);


                // Extend Arm Controls
//                if(intake.getState()) {
//                    robot.intake.pivotDown();
//                } else {
//                    robot.intake.pivotUp();
//                }

//                if(transfer.getState()) {
//                    robot.intake.closeDoor();
//                } else {
//                    robot.intake.openDoor();
//                }

                // Intake Controls
                if(LEFTTRIGGER1 > .2) {
                    robot.setIntake(-Constants.ExtendConstants.intakeSpeed);
                } else if (RIGHTTRIGGER1 > .2) {
                    robot.setIntake(Constants.ExtendConstants.intakeSpeed);
                } else {
                    robot.setIntake(0);
                }


                // Lifter Arm Controls
                if(DPAD_UP2) {
                    firstState = true;
                    state = States.PLACE;
                } else if (DPAD_LEFT2) {
                    firstState = true;
                    state = States.TRANSFER;
                } else if (DPAD_DOWN2) {
                    firstState = true;
                    state = States.INTAKE;
                }

                if(state == States.INTAKE) {
                    if(firstState == true) {
                        firstState = false;

                        doorTime.reset();
                    }
                    if(robot.lifter.leftArm.getPosition() == Constants.ClawConstants.liftArmIdle) {
                        if(robot.intake.intakePivot.getPosition() == Constants.IntakeConstants.intakePivotDown) {

                        } else {
                            if(robot.intake.door.getPosition() == Constants.IntakeConstants.doorClose && doorTime.milliseconds() >= 300) {
                                robot.intake.pivotDown();
                            } else {
                                robot.intake.closeDoor();
                            }
                        }
                    } else {
                        robot.lifter.leftArm.setPosition(Constants.ClawConstants.liftArmIdle);
                    }
                } else if (state == States.TRANSFER) {
                    if(firstState == true) {
                        firstState = false;

                        doorTime.reset();
                        pivotTime.reset();

                        robot.lifter.setWristPosition(Constants.ClawConstants.wristTransfer);
                        robot.updateLifterArm(Constants.ClawConstants.liftArmIdle);
                    }

                    if(robot.intake.intakePivot.getPosition() == Constants.IntakeConstants.intakePivotReset && pivotTime.milliseconds() >= 300) {
                        robot.lifter.clawOpen();

                        if(robot.intake.door.getPosition() == Constants.IntakeConstants.doorOpen && pivotTime.milliseconds() >= 800) {
                            if(robot.intake.door.getPosition() == Constants.IntakeConstants.doorOpen && pivotTime.milliseconds() >= 2400) {
                                robot.updateLifterArm(Constants.ClawConstants.liftArmReset);
                            }
                        } else if (pivotTime.milliseconds() >= 1900){
                            robot.intake.openDoor();
                        }
                    } else {
                        robot.lifter.setArmPosition(Constants.ClawConstants.liftArmIdle + .15f);
                        robot.intake.closeDoor();
                        robot.intake.pivotUp();
                    }
                } else if (state == States.PLACE) {
                    if(firstState == true) {
                        firstState = false;

                        doorTime.reset();
                        pivotTime.reset();
                    }
                    robot.lifter.clawClose();

                    if(pivotTime.milliseconds() >= 100) {
                        robot.updateLifterArm(Constants.ClawConstants.liftArmTop);
                        robot.updateLifterWrist(Constants.ClawConstants.wristDrop);
                    }

                    if(A2) {
                        robot.lifter.clawOpen();
                        doorTime.reset();
                        drop = true;
                    }

                    if(doorTime.milliseconds() >= 800 && drop) {
                        state = States.TRANSFER;

                        robot.updateLifterArm(Constants.ClawConstants.liftArmIdle);
                        robot.updateLifterWrist(Constants.ClawConstants.wristTransfer);
                        drop = false;
                    }

                    robot.update();
                }
            }
        }
    }

    public void updateInputs() {
        // Gamepad 1
        LY1 = -gamepad1.left_stick_y;
        LX1 = gamepad1.left_stick_x;

        RX1 = gamepad1.right_stick_x;

        A1 = gamepad1.a;
        B1 = gamepad1.b;

        Y1 = gamepad1.y;

        BACK1 = gamepad1.back;


        // Gamepad 2
        LY2 = -gamepad1.left_stick_y;

        RY2 = gamepad2.right_stick_y;

        LEFTTRIGGER1 = gamepad1.left_trigger;
        RIGHTTRIGGER1 = gamepad1.right_trigger;

        R2BUMPER = gamepad2.right_bumper;
        A2 = gamepad2.a;
        B2 = gamepad2.b;
        Y2 = gamepad2.y;
        X2 = gamepad2.x;

        DPAD_UP2 = gamepad2.dpad_up;
        DPAD_LEFT2 = gamepad2.dpad_left;
        DPAD_DOWN2 = gamepad2.dpad_down;


    }
}
