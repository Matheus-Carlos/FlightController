#include "utils.h"
#include "controllers.h"

const double Ix = 3.827e-3;
const double Iy = 3.827e-3;
const double Iz = 7.6566e-3;
const double Kphi = 5.567e-4;
const double Ktheta = 5.567e-4;
const double Kpsi = 5.567e-5;
const double Ir = 2.8385e-5;
const double g = 9.81;
const double m = 0.486;
const double Kx = 5.567e-4;
const double Ky = 5.567e-4;
const double Kz = 5.567e-5;

const double b = 2.9842e-3;
const double l = 0.25;
const double d = 3.232e-2;

double cascade_coversor_out[3];
double U[4] = { 5, 0, 0, 0 };
double inp_coversor_out[4] = { 0, 0, 0, 0 };

void inp_conversor() {
    static const double K[4][4] = {
        {0.25, 0., -0.5, 0.25},
        {0.25, -0.5, -0., -0.25},
        {0.25, 0., 0.5, 0.25},
        {0.25, 0.5, 0., -0.25}
    };
    static const double K2[4] = { b, l * b, l * b, d };
    
    for (int i = 0; i < 4; i++)
    {
        inp_coversor_out[i] = 0;
        for (int j = 0; j < 4; j++)
        {
            inp_coversor_out[i] += U[j] * K[i][j] / K2[j];
        }
        //inp_coversor_out[i] /= K2[i];
        inp_coversor_out[i] = sat(inp_coversor_out[i], 0, m * g / b / 2);
    }
}



AttitudePID::AttitudePID() {
    rot_pid[0] = PIDController(15, 0.01, 1.5);
    rot_pid[1] = PIDController(15, 0.01, 1.5);
    rot_pid[2] = PIDController(8, 1, 6);
}

void AttitudePID::calc(double X[], double dt) {
    for (int i = 0; i < 3; i++) {
        U[i + 1] = rot_pid[i].calc(X[i], 0.5, dt);
    };

    inp_conversor();
}

void calc_fs_and_gs_rot(double res[], double phi_dot, double theta_dot, double psi_dot) {
    double omega_r = 0;

    double f_phi = ((Iy - Iz) * theta_dot * psi_dot - Ir * omega_r * theta_dot - Kphi * phi_dot*phi_dot) / Ix;
    double f_theta = ((Iz - Ix) * phi_dot * psi_dot - Ir * omega_r * phi_dot - Ktheta * theta_dot * theta_dot) / Iy;
    double f_psi = ((Ix - Iy) * phi_dot * theta_dot - Kpsi * psi_dot * psi_dot) / Iz;

    res[0] = f_phi;
    res[1] = 1 / Ix;

    res[2] = f_theta;
    res[3] = 1 / Iy;

    res[4] = f_psi;
    res[5] = 1 / Iz;
}

AttitudeSMC::AttitudeSMC() {
    rot_smc[0] = SMCController(10, 100, 0.2);
    rot_smc[1] = SMCController(10, 100, 0.2);
    rot_smc[2] = SMCController(10, 100, 0.2);
}

void AttitudeSMC::calc(double X[], double dt) {
    double fs_and_gs[6];

    calc_fs_and_gs_rot(fs_and_gs, X[1], X[3], X[5]);

    for (int i = 0; i < 3; i++) {
        U[i + 1] = rot_smc[i].calc(X[2 * i], 0.5, dt, fs_and_gs[2*i], fs_and_gs[2 * i + 1]);
    };

    inp_conversor();
}





