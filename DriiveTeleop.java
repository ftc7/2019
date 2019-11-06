package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Gamepad;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;

class gpPrev1 {
    float left_stick_x = 0f;
    float left_stick_y = 0f;
    float right_stick_x = 0f;
    float right_stick_y = 0f;
    boolean dpad_up = false;
    boolean dpad_down = false;
    boolean dpad_left = false;
    boolean dpad_right = false;
    boolean a = false;
    boolean b = false;
    boolean x = false;
    boolean y = false;
    boolean start = false;
    boolean back = false;
    boolean guide = false;

    void update(Gamepad gamepad) {
        left_stick_x = gamepad.left_stick_x;
        left_stick_y = gamepad.left_stick_y;
        right_stick_x = gamepad.right_stick_x;
        right_stick_y = gamepad.right_stick_y;
        dpad_up = gamepad.dpad_up;
        dpad_down = gamepad.dpad_down;
        dpad_left = gamepad.dpad_left;
        dpad_right = gamepad.dpad_right;
        a = gamepad.a;
        b = gamepad.b;
        x = gamepad.x;
        y = gamepad.y;
        start = gamepad.start;
        back = gamepad.back;
        guide = gamepad.guide;
    }
}

@Config
class config {
    public static int servoPower = 0;
}

@TeleOp(name = "Driive v2")
public class DriiveTeleop extends OpMode {
    private DriivePrototypeHardware robot = new DriivePrototypeHardware();
    private gpPrev1 prev1 = new gpPrev1();
    private FtcDashboard dashboard = FtcDashboard.getInstance();
    private Driive driving = new Driive();

    public void init() {
        robot.init(hardwareMap);
        DcMotor[] wheels = {robot.one, robot.two, robot.three, robot.four};
        double[] angles = {0, Math.PI/2, Math.PI, Math.PI*3/2};
        try {
            driving.init(wheels, angles);
        } catch(Exception e) {
        }
        telemetry.addData("Initialized", "Initialized");
        telemetry.update();
    }

    public void init_loop() {
        driving.gyro(Math.toRadians(robot.angles.thirdAngle));
        TelemetryPacket packet =  new TelemetryPacket();
        driving.updateTelemetry(packet);
        dashboard.sendTelemetryPacket(packet);
    }

    public void loop() {
        robot.updateGyro(0);

        driving.cartesian(gamepad1.left_stick_x, gamepad1.left_stick_y, gamepad1.right_stick_x);
        driving.gyro(Math.toRadians(robot.angles.thirdAngle));
        driving.driive();

        if(gamepad1.x && !prev1.x) {
            driving.fieldCentric = !driving.fieldCentric;
        }
        if(gamepad1.start && !prev1.start) {
            driving.resetZero();
        }

        prev1.update(gamepad1);

        TelemetryPacket packet =  new TelemetryPacket();

        packet.put("fieldCentric", driving.fieldCentric);
        packet.put("righteous", driving.righteous);
        packet.put("speed", driving.speed);

        packet.put("thirdAngle", Math.toRadians(robot.angles.thirdAngle));

        driving.updateTelemetry(packet);

        dashboard.sendTelemetryPacket(packet);

/*        if(gamepad1.left_bumper) {
            robot.servo.setPower(100);
        } else if(gamepad1.right_bumper) {
            robot.servo.setPower(-1);
        } else {
            robot.servo.setPower(0);
        }*/
    }
}