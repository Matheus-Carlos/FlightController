#include "SMCController.h"
#include "utils.h"


SMCController::SMCController(double lamb_, double eta_, double mu_) {
    lamb = lamb_;
    eta = eta_;
    mu = mu_;
    x_prev = 0;
    xd_prev = 0;
    xd_dot_prev = 0;
}

double SMCController::calc(double x, double xd, double dt, double f, double g) {
    double x_til = x - xd;
    double xd_dot = (xd - xd_prev) / dt;
    double x_dot = (x - x_prev) / dt;
    double x_til_dot = x_dot - xd_dot;
    double xd_ddot = (xd_dot - xd_dot_prev) / dt;

    x_prev = x;
    xd_prev = xd;
    xd_dot_prev = xd_dot;

    double s = x_til_dot + lamb * x_til;

    return (
        xd_ddot
        - lamb * x_til_dot
        - f
        - eta * sat(s, -mu, mu)
        ) / g;
}