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

        // BUTTON TOGGLES
        FOD = new ButtonToggle();
        alignAprilTag = new ButtonToggle();

        antiTip = new ButtonToggleAdvanced(ButtonToggleAdvanced.NEUTRAL_STATE);

        intake = new ButtonToggle();
        transfer = new ButtonToggle();


        // TIMERS
        ElapsedTime stateTime = new ElapsedTime();
        ArrayList<Double> stateTimes = new ArrayList<>();

        double tempLastPosition = 0;


        States state = States.NONE;

        boolean firstState = true;


        waitForStart();
        if(opModeIsActive()) {
            while(opModeIsActive()) {

                updateInputs();

                // Controls
                FOD.update(A1);

                intake.update(Y2);
                transfer.update(X2);


                // Update Robot
                robot.updateDrive(LY1, LX1, RX1, FOD.getState(), false, alignAprilTag.getState(), antiTip);



                if (DPAD_UP2 || DPAD_LEFT2 || DPAD_DOWN2) {
                    firstState = true;
                    stateTimes = new ArrayList<>();

                    if(DPAD_UP2) {
                        state = States.PLACE;
                    } else if (DPAD_LEFT2) {
                        state = States.TRANSFER;
                    } else if (DPAD_DOWN2) {
                        state = States.INTAKE;
                    }
                }


                // Transfer State Machine
                switch(state) {

                    case INTAKE:

                        if(firstState) {
                            stateTime = new ElapsedTime();

                            robot.setLifter(Constants.LifterConstants.lifterZeroOffset);
                            robot.setLifterWrist(Constants.ClawConstants.wristTransfer);
                            robot.lifter.clawOpen();

                            robot.setExtend(robot.extend.currentExtendPosition);
                            robot.setIntake(0);


                            tempLastPosition = robot.lifter.currentArmPosition;

                            firstState = false;
                        }
                        

                        // Move arm to intake position
                        if(robot.lifter.currentArmPosition == Constants.LifterConstants.liftArmIntake &&
                                ((stateTimes.size() > 0) || (stateTime.milliseconds() >= MathUtil.calculateTimeMS(tempLastPosition, robot.lifter.currentArmPosition, Constants.TimerConstants.liftArmTimeMS)))) {
                            // Update Timer
                            if(stateTimes.size() == 0) {
                                stateTimes.add(stateTime.milliseconds());
                                tempLastPosition = robot.extend.currentIntakeDoorPosition;
                            }

                            // Close Door
                            if(robot.extend.currentIntakeDoorPosition == Constants.IntakeConstants.doorClose &&
                                    ((stateTimes.size() > 1) || (stateTime.milliseconds() - (double) (stateTimes.get(0)) >= MathUtil.calculateTimeMS(tempLastPosition, robot.extend.currentIntakeDoorPosition, Constants.TimerConstants.doorTimeMS)))) {
                                // Update Timer
                                if(stateTimes.size() == 1) {
                                    stateTimes.add(stateTime.milliseconds());
                                    tempLastPosition = robot.extend.currentArmPosition;
                                }

                                // Pivot Intake
                                if(robot.extend.currentArmPosition == Constants.ExtendConstants.intakeExtendArm &&
                                        ((stateTimes.size() > 2) || (stateTime.milliseconds() - (double) (stateTimes.get(1)) >= MathUtil.calculateTimeMS(tempLastPosition, robot.extend.currentArmPosition, Constants.TimerConstants.extendArmTimeMS)))) {
                                    // Update Timer
                                    if(stateTimes.size() == 2) {
                                        stateTimes.add(stateTime.milliseconds());
                                    }

                                    // MANUAL CONTROLS
                                        // Extend Controls
                                    if (!MathUtil.isInRange(LY2, Constants.OtherConstants.joystickThreshold)) {
                                        robot.setExtend((int) (LY2 * Constants.ExtendConstants.maxTicksPerLoop) + (robot.extend.currentExtendPosition));
                                    }

                                        // Intake Controls
                                    if(LEFTTRIGGER1 > Constants.OtherConstants.triggerThreshhold) {
                                        robot.setIntake(-Constants.IntakeConstants.intakeSpeed);
                                    } else if (RIGHTTRIGGER1 > Constants.OtherConstants.triggerThreshhold) {
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
                            robot.setLifterArm(Constants.LifterConstants.liftArmIntake);
                        }


                    case TRANSFER:

                        if(firstState) {
                            stateTime = new ElapsedTime();

                            robot.setLifter(Constants.LifterConstants.lifterZeroOffset);
                            robot.setLifterWrist(Constants.ClawConstants.wristTransfer);
                            robot.lifter.clawOpen();

                            robot.setExtend(robot.extend.currentExtendPosition);
                            robot.setIntake(0);


                            tempLastPosition = robot.extend.currentArmPosition;

                            firstState = false;
                        }


                        if(robot.lifter.currentArmPosition == Constants.LifterConstants.liftArmIntake &&
                                ((stateTimes.size() > 0) || (stateTime.milliseconds() >= MathUtil.calculateTimeMS(tempLastPosition, robot.lifter.currentArmPosition, Constants.TimerConstants.liftArmTimeMS)))) {
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
                                if (robot.extend.currentArmPosition == Constants.ExtendConstants.resetExtendArm &&
                                        ((stateTimes.size() > 2) || (stateTime.milliseconds() - (double) (stateTimes.get(1)) >= MathUtil.calculateTimeMS(tempLastPosition, robot.extend.currentArmPosition, Constants.TimerConstants.extendArmTimeMS)))) {
                                    // Update Timer
                                    if (stateTimes.size() == 2) {
                                        stateTimes.add(stateTime.milliseconds());
                                        tempLastPosition = robot.extend.currentIntakeDoorPosition;
                                    }

                                    // Open Door
                                    if(robot.extend.currentIntakeDoorPosition == Constants.IntakeConstants.doorOpen &&
                                            ((stateTimes.size() > 3) || (stateTime.milliseconds() - (double) (stateTimes.get(2)) >= MathUtil.calculateTimeMS(tempLastPosition, robot.extend.currentIntakeDoorPosition, Constants.TimerConstants.doorTimeMS)))) {
                                        // Update Timer
                                        if (stateTimes.size() == 3) {
                                            stateTimes.add(stateTime.milliseconds());
                                            tempLastPosition = robot.lifter.currentArmPosition;
                                        }

                                        // Move Arm to Transfer
                                        if(robot.lifter.currentArmPosition == Constants.LifterConstants.liftArmReset &&
                                                ((stateTimes.size() > 4) || (stateTime.milliseconds() - (double) (stateTimes.get(3)) >= MathUtil.calculateTimeMS(tempLastPosition, robot.lifter.currentArmPosition, Constants.TimerConstants.liftArmTimeMS)))) {
                                            // Update Timer
                                            if (stateTimes.size() == 4) {
                                                stateTimes.add(stateTime.milliseconds());
                                            }


                                            // STATE MACHINE READY

                                        } else {
                                            robot.lifter.setArmPosition(Constants.LifterConstants.liftArmReset);
                                        }
                                    } else {
                                        robot.extend.openIntakeDoor();
                                    }
                                } else {
                                    robot.setLifterArm(Constants.LifterConstants.liftArmIntake);
                                }
                            } else {
                                robot.setExtend(Constants.ExtendConstants.extendZeroOffset);
                            }
                        }


                    case PLACE:
                        if(firstState) {
                            stateTime = new ElapsedTime();

                            robot.setLifter(Constants.LifterConstants.lifterZeroOffset);
                            robot.setLifterWrist(Constants.ClawConstants.wristTransfer);
                            robot.lifter.clawOpen();

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

                                        // Update Timer
                                        if (stateTimes.size() == 1) {
                                            stateTimes.add(stateTime.milliseconds());
                                        }

                                    } else {
                                        robot.lifter.setWristPosition(Constants.ClawConstants.wristDrop);
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
                            if (!MathUtil.isInRange(LY2, Constants.OtherConstants.joystickThreshold)) {
                                robot.setLifter((int) (LY2 * Constants.LifterConstants.maxTicksPerLoop) + (robot.lifter.currentLiftPosition));
                            }

                                // Claw Controls
                            if(A2) {
                                if (stateTimes.size() == 2) {
                                    stateTimes.add(stateTime.milliseconds());
                                    tempLastPosition = robot.lifter.currentClawPosition;
                                }

                                robot.lifter.clawOpen();
                            }

                            if(stateTime.milliseconds() - (double) (stateTimes.get(2)) >= MathUtil.calculateTimeMS(tempLastPosition, robot.lifter.currentClawPosition, Constants.TimerConstants.clawTimeMS)) {
                                state = States.TRANSFER;

                                firstState = true;
                                stateTimes = new ArrayList<>();
                            }
                        }
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
