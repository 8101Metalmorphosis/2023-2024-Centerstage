package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.FTCutil.ButtonToggle;
import org.firstinspires.ftc.teamcode.FTCutil.ButtonToggleAdvanced;
import org.firstinspires.ftc.teamcode.FTCutil.MathUtil;
import org.firstinspires.ftc.teamcode.Subsystems.RobotBase;

import java.util.ArrayList;


@TeleOp(name = "State TeleOp", group = "!")
public class StateTeleOp extends LinearOpMode {

    RobotBase robot;

    // Button Toggles
    ButtonToggle FOD;
    ButtonToggle alignAprilTag;

    ButtonToggleAdvanced antiTip;


    ButtonToggle arm;
    ButtonToggle transfer;

    // Gamepad 1
    double LY1;
    double LX1;

    double RX1;

    boolean LEFTBUMPER1;
    boolean RIGHTBUMPER1;

    double LEFTTRIGGER1;
    double RIGHTTRIGGER1;

    boolean A1;
    boolean B1;

    boolean Y1;
    boolean X1;
    boolean BACK1;


    boolean DPAD_UP1;
    boolean DPAD_LEFT1;
    boolean DPAD_DOWN1;

    boolean START1;

    // Gamepad 2
//    double LY2;
//
//    double RY2;
//
//
//    boolean LEFTBUMPER2;
//    boolean RIGHTBUMPER2;
//
//    double LEFTTRIGGER2;
//    double RIGHTTRIGGER2;
//
//
//    boolean A2;
//    boolean B2;
//    boolean Y2;
//    boolean X2;
//
//    boolean DPAD_UP2;
//    boolean DPAD_LEFT2;
//    boolean DPAD_DOWN2;


    public enum States {
        INTAKE,
        TRANSFER,
        PLACE,
        NONE
    }

    @Override
    public void runOpMode() throws InterruptedException {

        robot = new RobotBase(hardwareMap, true);

        // BUTTON TOGGLES
        FOD = new ButtonToggle();
        alignAprilTag = new ButtonToggle();

        antiTip = new ButtonToggleAdvanced(ButtonToggleAdvanced.NEUTRAL_STATE);

        arm = new ButtonToggle();
        transfer = new ButtonToggle();


        // TIMERS
        ElapsedTime stateTime = new ElapsedTime();
        ArrayList<Double> stateTimes = new ArrayList<>();

        double tempLastPosition = 0;


        States state = States.NONE;

        boolean firstState = true;

        robot.lifter.zeroArm();

        while(opModeInInit()) {

            robot.setLifterArm(Constants.LifterConstants.liftArmTransfer);
            robot.setLifterWristPitch(Constants.ClawConstants.wristPitchTransfer);
            robot.setLifterWristRoll(Constants.ClawConstants.wristRollVertical);

            robot.extend.closeIntakeDoor();
            robot.setExtendArm(Constants.ExtendConstants.resetExtendArm);


            if((stateTime.milliseconds() > 500) && firstState){
                robot.lifter.resetArm();
                firstState = false;
            }
        }

        robot.lifter.resetArm();

        waitForStart();
        if(opModeIsActive()) {
            stateTime.reset();
            firstState = true;

            while(opModeIsActive()) {

                updateInputs();

                // Controls
                FOD.update(START1);

                arm.update(Y1);


                // Update Robot
                robot.updateDrive(LY1, LX1, RX1, FOD.getState(), false, alignAprilTag.getState(), antiTip);


                if (DPAD_UP1 || DPAD_LEFT1 || DPAD_DOWN1) {
                    firstState = true;
                    stateTimes = new ArrayList<>();

                    if(DPAD_UP1) {
                        state = States.PLACE;
                    } else if (DPAD_LEFT1) {
                        state = States.TRANSFER;
                    } else if (DPAD_DOWN1) {
                        state = States.INTAKE;
                    }
                }

                // Transfer State Machine

                switch(state) {

                    case INTAKE:

                        if(firstState) {
                            stateTime = new ElapsedTime();

                            robot.setLifter(Constants.LifterConstants.lifterMinHeight);
                            robot.setLifterWristPitch(Constants.ClawConstants.wristPitchTransfer);
                            robot.lifter.clawOpen();

                            robot.setExtend(robot.extend.currentExtendPosition);
                            robot.setIntake(0);


                            tempLastPosition = robot.lifter.currentArmPosition;

                            firstState = false;
                        }


                        // Move arm to intake position
                        if((robot.lifter.currentArmPosition == Constants.LifterConstants.liftArmIdle &&
                                (stateTime.milliseconds() >= MathUtil.calculateTimeMS(tempLastPosition, robot.lifter.currentArmPosition, Constants.TimerConstants.liftArmTimeMS))) ||
                                ((stateTimes.size() > 0))) {
                            // Update Timer
                            if(stateTimes.size() == 0) {
                                stateTimes.add(stateTime.milliseconds());
                                tempLastPosition = robot.extend.currentIntakeDoorPosition;
                            }

                            // Close Door
                            if((robot.extend.currentIntakeDoorPosition == Constants.IntakeConstants.doorClose &&
                                    (stateTime.milliseconds() - (double) (stateTimes.get(0)) >= MathUtil.calculateTimeMS(tempLastPosition, robot.extend.currentIntakeDoorPosition, Constants.TimerConstants.doorTimeMS))) ||
                                ((stateTimes.size() > 1))) {
                                // Update Timer
                                if(stateTimes.size() == 1) {
                                    stateTimes.add(stateTime.milliseconds());
                                    tempLastPosition = robot.extend.currentArmPosition;
                                }

                                // Pivot Intake
                                if((robot.extend.currentArmPosition == Constants.ExtendConstants.intakeExtendArm &&
                                        (stateTime.milliseconds() - (double) (stateTimes.get(1)) >= MathUtil.calculateTimeMS(tempLastPosition, robot.extend.currentArmPosition, Constants.TimerConstants.extendArmTimeMS))) ||
                                        ((stateTimes.size() > 2))) {
                                    // Update Timer
                                    if(stateTimes.size() == 2) {
                                        stateTimes.add(stateTime.milliseconds());
                                    }

                                    robot.setLifterArm(Constants.LifterConstants.liftArmIntake);

                                    // MANUAL CONTROLS
                                        // Extend Controls
                                    if (!MathUtil.isInRange(LEFTTRIGGER1, Constants.OtherConstants.joystickThreshold) || !MathUtil.isInRange(RIGHTTRIGGER1, Constants.OtherConstants.joystickThreshold)) {
                                        robot.setExtend(
                                                (int) MathUtil.putInRange(Constants.ExtendConstants.extendMinHeight,
                                                        (int) ((RIGHTTRIGGER1 - LEFTTRIGGER1) * Constants.ExtendConstants.maxTicksPerLoop) + (robot.lifter.currentLiftPosition),
                                                        Constants.ExtendConstants.extendMaxHeight));
                                    }

                                        // Intake Controls
                                    if(LEFTBUMPER1) {
                                        robot.setIntake(-Constants.IntakeConstants.intakeSpeed);
                                    } else if (RIGHTBUMPER1) {
                                        robot.setIntake(Constants.IntakeConstants.intakeSpeed);
                                    } else {
                                        robot.setIntake(0);
                                    }
                                } else {
                                    robot.extend.pivotArmDown();
                                }
                            } else {
                                robot.extend.closeIntakeDoor();
                            }
                        } else {
                            robot.setLifterArm(Constants.LifterConstants.liftArmIdle);
                        }
                    break;



                    case TRANSFER:


                        if(firstState) {
                            stateTime = new ElapsedTime();

                            robot.setLifter(Constants.LifterConstants.lifterMinHeight);
                            robot.setLifterWristPitch(Constants.ClawConstants.wristPitchTransfer);
                            robot.lifter.setWristRollPosition(Constants.ClawConstants.wristRollVertical);
                            robot.lifter.clawOpen();

                            robot.setExtend(robot.extend.currentExtendPosition);
                            robot.setIntake(0);


                            tempLastPosition = robot.extend.currentArmPosition;

                            firstState = false;
                        }


                        if((robot.lifter.currentArmPosition == Constants.LifterConstants.liftArmIdle &&
                                stateTime.milliseconds() >= MathUtil.calculateTimeMS(tempLastPosition, robot.lifter.currentArmPosition, Constants.TimerConstants.liftArmTimeMS)) ||
                                (stateTimes.size() > 0)) {
                            // Update Timer
                            if (stateTimes.size() == 0) {
                                stateTimes.add(stateTime.milliseconds());
                            }

                            // Unextend
                            if (MathUtil.isInRange(robot.extend.currentExtendPosition, Constants.ExtendConstants.extendAllowedThreshold)) {
                                // Update Timer
                                if (stateTimes.size() == 1) {
                                    stateTimes.add(stateTime.milliseconds());
                                    tempLastPosition = robot.extend.currentArmPosition;
                                }

                                // Pivot Intake
                                if ((robot.extend.currentArmPosition == Constants.ExtendConstants.resetExtendArm &&
                                        stateTime.milliseconds() - (double) (stateTimes.get(1)) >= MathUtil.calculateTimeMS(tempLastPosition, robot.extend.currentArmPosition, Constants.TimerConstants.extendArmTimeMS)) ||
                                        (stateTimes.size() > 2)) {
                                    // Update Timer
                                    if (stateTimes.size() == 2) {
                                        stateTimes.add(stateTime.milliseconds());
                                        tempLastPosition = robot.extend.currentIntakeDoorPosition;
                                    }

                                    // Open Door
                                    if((robot.extend.currentIntakeDoorPosition == Constants.IntakeConstants.doorOpen &&
                                            (stateTime.milliseconds() - (double) (stateTimes.get(2)) >= MathUtil.calculateTimeMS(tempLastPosition, robot.extend.currentIntakeDoorPosition, Constants.TimerConstants.doorTimeMS))) ||
                                            (stateTimes.size() > 3)) {
                                        // Update Timer
                                        if (stateTimes.size() == 3) {
                                            stateTimes.add(stateTime.milliseconds());
                                            tempLastPosition = robot.lifter.currentArmPosition;
                                        }

                                        // Move Arm to Transfer
                                        if((robot.lifter.currentArmPosition == Constants.LifterConstants.liftArmTransfer &&
                                                (stateTime.milliseconds() - (double) (stateTimes.get(3)) >= MathUtil.calculateTimeMS(tempLastPosition, robot.lifter.currentArmPosition, Constants.TimerConstants.liftArmTimeMS))) ||
                                                (stateTimes.size() > 4)) {
                                            // Update Timer
                                            if (stateTimes.size() == 4) {
                                                stateTimes.add(stateTime.milliseconds());
                                            }


                                            // STATE MACHINE READY

                                        } else {
                                            robot.lifter.setArmPosition(Constants.LifterConstants.liftArmTransfer);
                                        }
                                    } else {
                                        robot.extend.openIntakeDoor();
                                    }
                                } else {
                                    robot.setExtendArm(Constants.ExtendConstants.resetExtendArm);
                                }
                            } else {
                                robot.setExtend(Constants.ExtendConstants.extendMinHeight);
                            }
                        } else {
                            robot.setLifterArm(Constants.LifterConstants.liftArmIdle);
                        }

                    break;



                    case PLACE:
                            if(firstState) {
                                stateTime = new ElapsedTime();

                                robot.setLifter(Constants.LifterConstants.lifterMinHeight);
                                robot.setLifterWristPitch(Constants.ClawConstants.wristPitchTransfer);
                                robot.lifter.clawClose();

                                robot.setExtend(robot.extend.currentExtendPosition);
                                robot.setIntake(0);

                                firstState = false;
                            }

                            // if needs to run through state machine
                            if (stateTimes.size() != 2 && stateTimes.size() != 3) {
                                // Close Claw
                                if(robot.lifter.currentClawPosition == Constants.ClawConstants.clawClose) {
                                    // Update Timer
                                    if (stateTimes.size() == 0) {
                                        stateTimes.add(stateTime.milliseconds());
                                        tempLastPosition = robot.lifter.currentArmPosition;
                                    }

                                    // Rotate Arm to main position
                                    if(robot.lifter.currentArmPosition == Constants.LifterConstants.liftArmTop && stateTime.milliseconds() - (double) (stateTimes.get(0)) >= 250 && stateTimes.size() == 1) {
                                        if ((stateTime.milliseconds() - (double) (stateTimes.get(0)) >= MathUtil.calculateTimeMS(tempLastPosition, robot.lifter.currentArmPosition, Constants.TimerConstants.liftArmTimeMS))) {
                                            robot.lifter.setWristPitchPosition(Constants.ClawConstants.wristPitchDrop);

                                            // Update Timer
                                            if (stateTimes.size() == 1) {
                                                stateTimes.add(stateTime.milliseconds());
                                            }

                                        } else {
                                            robot.lifter.setWristPitchPosition(Constants.ClawConstants.wristPitchDrop);
                                        }
                                    } else {
                                        robot.lifter.setArmPosition(Constants.LifterConstants.liftArmTop);
                                    }
                                } else {
                                    robot.lifter.clawClose();
                                }
                            } else {
                                // MANUAL CONTROLS
                                // Lifter Controls

                                if(!arm.getState()) {
                                    if (!MathUtil.isInRange(LEFTTRIGGER1, Constants.OtherConstants.joystickThreshold) || !MathUtil.isInRange(RIGHTTRIGGER1, Constants.OtherConstants.joystickThreshold)) {
                                        robot.setLifter(
                                                (int) MathUtil.putInRange(Constants.LifterConstants.lifterMinHeight,
                                                        (int) (((double) (RIGHTTRIGGER1 - LEFTTRIGGER1) * Constants.LifterConstants.maxTicksPerLoop) + (robot.lifter.currentLiftPosition)),
                                                        Constants.LifterConstants.lifterMaxHeight));
                                    }
                                    robot.setLifterArm(Constants.LifterConstants.liftArmTop);
                                    robot.setLifterWristPitch(Constants.ClawConstants.wristPitchDrop);
                                } else {
                                    robot.setLifterArm(Constants.LifterConstants.liftArmTop2);
                                    robot.setLifterWristPitch(Constants.ClawConstants.wristPitchDrop2);
                                }


                                // Wrist Controls
                                if(X1) {
                                    robot.lifter.setWristRollPosition(Constants.ClawConstants.wristRollLeft);
                                } else if (B1) {
                                    robot.lifter.setWristRollPosition(Constants.ClawConstants.wristRollRight);
                                } else {
                                    robot.lifter.setWristRollPosition(Constants.ClawConstants.wristRollVertical);
                                }

                                // Claw Controls
                                if(A1) {
                                    if (stateTimes.size() == 2) {
                                        stateTimes.add(stateTime.milliseconds());
                                        tempLastPosition = robot.lifter.currentClawPosition;

                                        robot.lifter.claw.setPosition(Constants.ClawConstants.clawFullOpen); // USED TO BE CLAW FULL OPEN AS OF 2/1/2024
                                    }
                                }

                                if(stateTimes.size() == 3) {
                                    if(stateTime.milliseconds() - (double) (stateTimes.get(2)) >= MathUtil.calculateTimeMS(tempLastPosition, robot.lifter.currentClawPosition, Constants.TimerConstants.clawTimeMS) + 1000) {
                                        state = States.TRANSFER;

                                        arm.setState(false);

                                        firstState = true;
                                        stateTimes = new ArrayList<>();
                                    }
                                }
                            }

                    break;
                }
                robot.update();
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
        X1 = gamepad1.x;

        BACK1 = gamepad1.back;
        START1 = gamepad1.start;

        LEFTBUMPER1 = gamepad1.left_bumper;
        RIGHTBUMPER1 = gamepad1.right_bumper;

        LEFTTRIGGER1 = gamepad1.left_trigger;
        RIGHTTRIGGER1 = gamepad1.right_trigger;

        DPAD_UP1 = gamepad1.dpad_up;
        DPAD_LEFT1 = gamepad1.dpad_left;
        DPAD_DOWN1 = gamepad1.dpad_down;


        // Gamepad 2
//        LY2 = -gamepad2.left_stick_y;
//
//        RY2 = gamepad2.right_stick_y;
//
//        LEFTTRIGGER2 = gamepad2.left_trigger;
//        RIGHTTRIGGER2 = gamepad2.right_trigger;
//
//        LEFTBUMPER2 = gamepad2.left_bumper;
//        RIGHTBUMPER2 = gamepad2.right_bumper;
//
//        A2 = gamepad2.a;
//        B2 = gamepad2.b;
//        Y2 = gamepad2.y;
//        X2 = gamepad2.x;
//
//        DPAD_UP2 = gamepad2.dpad_up;
//        DPAD_LEFT2 = gamepad2.dpad_left;
//        DPAD_DOWN2 = gamepad2.dpad_down;
    }
}