#ifndef SMCCONTROLLER_H
	#define SMCCONTROLLER_H

	class SMCController {
	private:
		double lamb, eta, mu, x_prev, xd_prev, xd_dot_prev;

	public:
		SMCController(double lamb_ = 1, double eta_ = 1, double mu_ = 1);
		double calc(double x, double xd, double dt, double f, double g);
	};
#endif // !SMCCONTROLLER_H
