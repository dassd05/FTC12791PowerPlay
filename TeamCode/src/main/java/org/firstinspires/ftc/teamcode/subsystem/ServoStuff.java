package org.firstinspires.ftc.teamcode.subsystem;

import com.qualcomm.robotcore.hardware.PwmControl;

@SuppressWarnings("unused")
public interface ServoStuff {
    interface RevSmartRobotServo {
        double noLoadSpeed48Volt = -1;  // sec/60deg
        double stallTorque48Volt = -1;  // kg-cm
        double noLoadCurrent48Volt = -1;  // mA
        double stallCurrent48Volt = -1;  // mA

        double noLoadSpeed6Volt = .14;  // sec/60deg
        double stallTorque6Volt = 13.5;  // kg-cm
        double noLoadCurrent6Volt = -1;  // mA
        double stallCurrent6Volt = 2000;  // mA

        double noLoadSpeed74Volt = -1;  // sec/60deg
        double stallTorque74Volt = -1;  // kg-cm
        double noLoadCurrent74Volt = -1;  // mA
        double stallCurrent74Volt = -1;  // mA

        double defaultAngleRange = 270;  // deg
        double maxAngleRange = 280;  // deg
        PwmControl.PwmRange servoModePwmRange = new PwmControl.PwmRange(500, 2500);
        PwmControl.PwmRange continuousModePwmRange = new PwmControl.PwmRange(500, 2500);  // i think

        boolean directionIncreasingClockwise = Boolean.parseBoolean(null);
    }

    interface GobildaTorqueServo {
        double noLoadSpeed48Volt = .25;  // sec/60deg
        double stallTorque48Volt = 17.2;  // kg-cm
        double noLoadCurrent48Volt = 150;  // mA
        double stallCurrent48Volt = 2000;  // mA

        double noLoadSpeed6Volt = .20;  // sec/60deg
        double stallTorque6Volt = 21.6;  // kg-cm
        double noLoadCurrent6Volt = 160;  // mA
        double stallCurrent6Volt = 2500;  // mA

        double noLoadSpeed74Volt = .17;  // sec/60deg
        double stallTorque74Volt = 25.2;  // kg-cm
        double noLoadCurrent74Volt = 200;  // mA
        double stallCurrent74Volt = 3000;  // mA

        // maybe 280 idk servo programmer says smth different
        double defaultAngleRange = 300;  // deg
        double maxAngleRange = 300;  // deg

        PwmControl.PwmRange servoModePwmRange = new PwmControl.PwmRange(500, 2500);
        PwmControl.PwmRange continuousModePwmRange = new PwmControl.PwmRange(900, 2100);

        boolean directionIncreasingClockwise = true;
    }

    interface GobildaSpeedServo {
        double noLoadSpeed48Volt = .11;  // sec/60deg
        double stallTorque48Volt = 7.9;  // kg-cm
        double noLoadCurrent48Volt = 190;  // mA
        double stallCurrent48Volt = 2000;  // mA

        double noLoadSpeed6Volt = .09;  // sec/60deg
        double stallTorque6Volt = 9.3;  // kg-cm
        double noLoadCurrent6Volt = 200;  // mA
        double stallCurrent6Volt = 2500;  // mA

        double noLoadSpeed74Volt = .07;  // sec/60deg
        double stallTorque74Volt = 10.8;  // kg-cm
        double noLoadCurrent74Volt = 230;  // mA
        double stallCurrent74Volt = 3000;  // mA

        // maybe 280 idk servo programmer says smth different
        double defaultAngleRange = 300;  // deg
        double maxAngleRange = 300;  // deg

        PwmControl.PwmRange servoModePwmRange = new PwmControl.PwmRange(500, 2500);
        PwmControl.PwmRange continuousModePwmRange = new PwmControl.PwmRange(1000, 2000);

        boolean directionIncreasingClockwise = true;
    }

    interface GobildaSuperSpeecServo {
        double noLoadSpeed48Volt = .055;  // sec/60deg
        double stallTorque48Volt = 4.0;  // kg-cm
        double noLoadCurrent48Volt = 190;  // mA
        double stallCurrent48Volt = 2000;  // mA

        double noLoadSpeed6Volt = .043;  // sec/60deg
        double stallTorque6Volt = 4.7;  // kg-cm
        double noLoadCurrent6Volt = 200;  // mA
        double stallCurrent6Volt = 2500;  // mA

        double noLoadSpeed74Volt = .035;  // sec/60deg
        double stallTorque74Volt = 5.4;  // kg-cm
        double noLoadCurrent74Volt = 230;  // mA
        double stallCurrent74Volt = 3000;  // mA

        // maybe 280 idk servo programmer says smth different
        double defaultAngleRange = 300;  // deg
        double maxAngleRange = 300;  // deg

        PwmControl.PwmRange servoModePwmRange = new PwmControl.PwmRange(500, 2500);
        PwmControl.PwmRange continuousModePwmRange = new PwmControl.PwmRange(1000, 2000);

        boolean directionIncreasingClockwise = true;
    }

    interface AxonMaxServo {
        double noLoadSpeed48Volt = .14;  // sec/60deg
        double stallTorque48Volt = 28;  // kg-cm
        double noLoadCurrent48Volt = 280;  // mA
        double stallCurrent48Volt = 3600;  // mA

        double noLoadSpeed6Volt = .115;  // sec/60deg
        double stallTorque6Volt = 34;  // kg-cm
        double noLoadCurrent6Volt = 320;  // mA
        double stallCurrent6Volt = 4000;  // mA

        double noLoadSpeed74Volt = .100;  // sec/60deg
        double stallTorque74Volt = 39;  // kg-cm
        double noLoadCurrent74Volt = 360;  // mA
        double stallCurrent74Volt = 4300;  // mA

        // maybe 280 idk servo programmer says smth different
        double defaultAngleRange = 180;  // deg
        double maxAngleRange = 355;  // deg

        PwmControl.PwmRange servoModePwmRange = new PwmControl.PwmRange(500, 2500);
        PwmControl.PwmRange continuousModePwmRange = new PwmControl.PwmRange(500, 2500);

        boolean directionIncreasingClockwise = false;
    }

    interface AxonMiniServo {
        double noLoadSpeed48Volt = .110;  // sec/60deg
        double stallTorque48Volt = 20;  // kg-cm
        double noLoadCurrent48Volt = 300;  // mA
        double stallCurrent48Volt = 3200;  // mA

        double noLoadSpeed6Volt = .090;  // sec/60deg
        double stallTorque6Volt = 25;  // kg-cm
        double noLoadCurrent6Volt = 350;  // mA
        double stallCurrent6Volt = 3800;  // mA

        double noLoadSpeed74Volt = .080;  // sec/60deg
        double stallTorque74Volt = 30;  // kg-cm
        double noLoadCurrent74Volt = 380;  // mA
        double stallCurrent74Volt = 4300;  // mA

        // maybe 280 idk servo programmer says smth different
        double defaultAngleRange = 180;  // deg
        double maxAngleRange = 355;  // deg

        PwmControl.PwmRange servoModePwmRange = new PwmControl.PwmRange(500, 2500);
        PwmControl.PwmRange continuousModePwmRange = new PwmControl.PwmRange(500, 2500);

        boolean directionIncreasingClockwise = false;
    }
}
