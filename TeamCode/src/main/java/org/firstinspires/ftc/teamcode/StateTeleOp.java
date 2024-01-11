package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

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
    boolean DPAD_DOWN2;

    @Override
    public void runOpMode() throws InterruptedException {

        robot = new RobotBase(hardwareMap);

        FOD = new ButtonToggle();
        alignAprilTag = new ButtonToggle();

        antiTip = new ButtonToggleAdvanced(ButtonToggleAdvanced.NEUTRAL_STATE);

        intake = new ButtonToggle();
        transfer = new ButtonToggle();

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


                // Lifter Arm Controls
                if(DPAD_UP2) {
                    robot.updateLifterArm(Constants.LifterConstants.topLifterArm);
                } else if (DPAD_DOWN2) {
                    robot.updateLifterArm(Constants.LifterConstants.topLifterArm);
                }

                // Extend Arm Controls
                if(intake.getState()) {
                    robot.intake.pivotDown();
                } else {
                    robot.intake.pivotUp();
                }

                if(transfer.getState()) {
                    robot.intake.closeDoor();
                } else {
                    robot.intake.openDoor();
                }

                // Intake Controls
                if(A1) {
                    robot.setIntake(Constants.ExtendConstants.intakeSpeed);
                } else if (B1) {
                    robot.setIntake(-Constants.ExtendConstants.intakeSpeed);
                } else {
                    robot.setIntake(0);
                }


//                if(!MathUtil.isInRange(LY2, .1)) {
//                    robot.updateLifter((int) (robot.lifter.leftPos + (400 * LY2)));
//                }
//
//                // Manual Extend Controls
//                if(!MathUtil.isInRange(RY2, .1)) {
//                    robot.updateExtend(robot.extend.extendPos);
//                }


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
        R2BUMPER = gamepad2.right_bumper;
        A2 = gamepad2.a;
        B2 = gamepad2.b;
        Y2 = gamepad2.y;
        X2 = gamepad2.x;

        DPAD_UP2 = gamepad2.dpad_up;
        DPAD_DOWN2 = gamepad2.dpad_down;


    }
}
