#ifndef CONTROLLERS_H
	#define CONTROLLERS_H

	#include "SMCController.h"
	#include "PIDController.h"

	extern boost::array< double, 4 > inp_coversor_out;
	extern boost::array< double, 4 > U;

	class CascadePID {
	private:
		PIDController pos_pid[3];
		PIDController rot_pid[3];

	public:
		CascadePID();
		void calc(double X[], double dt);
	};

	class AttitudePID {
	private:
		PIDController rot_pid[3];

	public:
		AttitudePID();
		void calc(double X[], double dt);
	};

	class AttitudeSMC {
	private:
		SMCController rot_smc[3];

	public:
		AttitudeSMC();
		void calc(double X[], double dt);
	};
#endif // !CONTROLLERS_H

