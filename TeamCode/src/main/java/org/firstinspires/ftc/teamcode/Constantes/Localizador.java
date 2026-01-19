package org.firstinspires.ftc.teamcode.Constantes;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.IMU;

public class Localizador {
    IMU imu;
    DcMotor encDir, encEsq, encPerp;
    double raioDir, raioEsq, raioPerp, ticksperCM;
    public Localizador(HardwareMap hardwareMap, String encoderDirName, String encoderEsqNome, String encoderPerpNome){
        this.imu = hardwareMap.get(IMU.class,"imu");
        this.encDir = hardwareMap.dcMotor.get(encoderDirName);
        this.encEsq = hardwareMap.dcMotor.get(encoderEsqNome);
        this.encPerp = hardwareMap.dcMotor.get(encoderPerpNome);
    }
    void setarConstantes(double distanciaXdir, double distanciaXesq, double distanciaYperp, double ticksperCM){
        this.raioDir = distanciaXdir;
        this.raioEsq = distanciaXesq;
        this.raioPerp = distanciaYperp;
        this.ticksperCM = ticksperCM;
    }
}
