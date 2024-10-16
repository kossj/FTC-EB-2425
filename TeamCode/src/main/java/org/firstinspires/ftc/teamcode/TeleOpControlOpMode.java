/* Copyright (c) 2017 FIRST. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted (subject to the limitations in the disclaimer below) provided that
 * the following conditions are met:
 *
 * Redistributions of source code must retain the above copyright notice, this list
 * of conditions and the following disclaimer.
 *
 * Redistributions in binary form must reproduce the above copyright notice, this
 * list of conditions and the following disclaimer in the documentation and/or
 * other materials provided with the distribution.
 *
 * Neither the name of FIRST nor the names of its contributors may be used to endorse or
 * promote products derived from this software without specific prior written permission.
 *
 * NO EXPRESS OR IMPLIED LICENSES TO ANY PARTY'S PATENT RIGHTS ARE GRANTED BY THIS
 * LICENSE. THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
 * THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.util.ElapsedTime;

/*
 * This file contains an example of an iterative (Non-Linear) "OpMode".
 * An OpMode is a 'program' that runs in either the autonomous or the teleop period of an FTC match.
 * The names of OpModes appear on the menu of the FTC Driver Station.
 * When a selection is made from the menu, the corresponding OpMode
 * class is instantiated on the Robot Controller and executed.
 *
 * This particular OpMode just executes a basic Tank Drive Teleop for a two wheeled robot
 * It includes all the skeletal structure that all iterative OpModes contain.
 *
 * Use Android Studio to Copy this Class, and Paste it into your team's code folder with a new name.
 * Remove or comment out the @Disabled line to add this OpMode to the Driver Station OpMode list
 */

@TeleOp(name="TeleOp Control", group="Teleop")

public class TeleOpControlOpMode extends OpMode
{
    // Declare OpMode members.
    private ElapsedTime runtime = new ElapsedTime();

    // Declare drive motors
    private CRServo leftFrontDrive = null;
    private DcMotor leftBackDrive = null;
    private CRServo rightFrontDrive = null;
    private DcMotor rightBackDrive = null;

    // Declare end-effector members
    private CRServo intake = null;
    private DcMotor extension = null;
    private DcMotorEx pivot = null;

    private final double INTAKE_IN_POWER = 1.0;
    private final double INTAKE_OUT_POWER = -1.0;
    private final double INTAKE_OFF_POWER = 0.0;

    private double intakePower = INTAKE_OFF_POWER;


    private final double EXTENSION_OUT_POWER = 1.0;
    private final double EXTENSION_IN_POWER = -1.0;

    private boolean was_holding = false;

    private int pivot_target_pos;

    @Override
    public void init() {
        telemetry.addData("Status", "Initialized");

        leftFrontDrive  = hardwareMap.get(CRServo.class, "left_front_drive");
        leftBackDrive  = hardwareMap.get(DcMotor.class, "left_back_drive");
        rightFrontDrive = hardwareMap.get(CRServo.class, "right_front_drive");
        rightBackDrive = hardwareMap.get(DcMotor.class, "right_back_drive");

        intake = hardwareMap.get(CRServo.class, "intake");
        extension = hardwareMap.get(DcMotor.class, "extension");
        pivot = hardwareMap.get(DcMotorEx.class, "pivot");

        // TODO: Make sure all motors are facing the correct direction. Go one at a time.
        leftFrontDrive.setDirection(CRServo.Direction.REVERSE);
        leftBackDrive.setDirection(DcMotor.Direction.REVERSE);
        rightFrontDrive.setDirection(CRServo.Direction.FORWARD);
        rightBackDrive.setDirection(DcMotor.Direction.FORWARD);

        intake.setDirection(CRServo.Direction.FORWARD); // Forward should INTAKE.
        extension.setDirection(DcMotor.Direction.REVERSE); // Forward should EXTEND.
        pivot.setDirection(DcMotor.Direction.REVERSE); // Forward should pivot UP, or away from the stowed position.

        extension.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        pivot.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        pivot.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        telemetry.addData("Status", "Initialized");
        telemetry.update();
    }

    @Override
    public void init_loop() {
    }

    @Override
    public void start() {
        runtime.reset();
    }

    @Override
    public void loop() {
        double max;

        // COLLECT INPUTS
        // POV Mode uses left joystick to go forward & strafe, and right joystick to rotate.
        double axial   = -gamepad1.left_stick_y;  // Note: pushing stick forward gives negative value
        double lateral =  -gamepad1.right_stick_x;
        double yaw     =  -gamepad1.left_stick_x;

        boolean intakeInButton = gamepad1.a;
        boolean intakeOutButton = gamepad1.b;
        boolean intakeOffButton = gamepad1.x;
        // This conditional reduces ambiguity when multiple buttons are pressed.
        if (intakeInButton && intakeOutButton) {
            intakeInButton = false;
        } else if (intakeOffButton && (intakeInButton || intakeOutButton)) {
            intakeInButton = intakeOutButton = false;
        }

        boolean extensionOutButton = gamepad1.left_trigger > 0.2;
        boolean extensionInButton = gamepad1.left_bumper;
        if (extensionOutButton && extensionInButton) {
            extensionOutButton = false;
        }

        boolean pivotUpButton = gamepad1.right_bumper;
        boolean pivotDownButton = gamepad1.right_trigger > 0.2;
        if (pivotUpButton && pivotDownButton) {
            pivotUpButton = false;
        }

        // DRIVE CODE
        // Combine the joystick requests for each axis-motion to determine each wheel's power.
        // Set up a variable for each drive wheel to save the power level for telemetry.
        double leftFrontPower  = axial + lateral + yaw;
        double rightFrontPower = axial - lateral - yaw;
        double leftBackPower   = axial - lateral + yaw;
        double rightBackPower  = axial + lateral - yaw;

        // Normalize the values so no wheel power exceeds 100%
        // This ensures that the robot maintains the desired motion.
        max = Math.max(Math.abs(leftFrontPower), Math.abs(rightFrontPower));
        max = Math.max(max, Math.abs(leftBackPower));
        max = Math.max(max, Math.abs(rightBackPower));

        if (max > 1.0) {
            leftFrontPower  /= max;
            rightFrontPower /= max;
            leftBackPower   /= max;
            rightBackPower  /= max;
        }

        // INTAKE CODE
        if (intakeInButton) {
            intakePower = INTAKE_IN_POWER;
        } else if (intakeOutButton) {
            intakePower = INTAKE_OUT_POWER;
        } else if (intakeOffButton) {
            intakePower = 0.0;
        }

        // EXTENSION CODE
        double extensionPower;
        if (extensionOutButton) {
            extensionPower = EXTENSION_OUT_POWER;
        } else if (extensionInButton) {
            extensionPower = EXTENSION_IN_POWER;
        } else {
            extensionPower = 0;
        }

        String pivot_mode_str; // Used for telemetry
        // Determine pivot mode
        if (pivotUpButton) {
            was_holding = false;
            pivot.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            pivot.setPower(0.75);
            pivot_mode_str = "UP";
        } else if (pivotDownButton) {
            was_holding = false;
            pivot.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            pivot.setPower(-0.75);
            pivot_mode_str = "DOWN";
        } else {
            if (!was_holding) {
                pivot_target_pos = pivot.getCurrentPosition();
                was_holding = true;
            }
            pivot.setTargetPosition(pivot_target_pos);
            pivot.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            pivot.setPower(0.75);
            pivot_mode_str = "HOLD";
        }

        // WRITE EFFECTORS
        leftFrontDrive.setPower(leftFrontPower);
        rightFrontDrive.setPower(rightFrontPower);
        leftBackDrive.setPower(leftBackPower);
        rightBackDrive.setPower(rightBackPower);


        intake.setPower(intakePower);
        extension.setPower(extensionPower);

        // UPDATE TELEMETRY
        telemetry.addData("Status", "Run Time: " + runtime.toString());
        telemetry.addData("Front left/Right", "%4.2f, %4.2f", leftFrontPower, rightFrontPower);
        telemetry.addData("Back  left/Right", "%4.2f, %4.2f", leftBackPower, rightBackPower);
        telemetry.addData("Intake", "%%4.2f", intakePower);
        telemetry.addData("Extension", "%4.2f", extension.getPower());
        telemetry.addData("Pivot Current/Target/power", "%d, %d, %4.2f", pivot.getCurrentPosition(), pivot.getTargetPosition(),pivot.getPower());
        telemetry.addData("Pivot MODE", "%s", pivot_mode_str);
        telemetry.update();
    }


    @Override
    public void stop() {
    }

}
