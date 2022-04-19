#ifndef CONTROLLERS_H
	#define CONTROLLERS_H

	#include "SMCController.h"
	#include "PIDController.h"

	extern boost::array< double, 4 >u inp_coversor_out;
	extern boost::array< double, 4 >u U;

	class CascadePID {
	private:
		PIDController pos_pid[3];
		PIDController rot_pid[3];

	public:
		CascadePID();
		void calc(state_type& X, double dt);
	};

	class AttitudePID {
	private:
		PIDController rot_pid[3];

	public:
		AttitudePID();
		void calc(state_type& X, double dt);
	};

	class AttitudeSMC {
	private:
		SMCController rot_smc[3];

	public:
		AttitudeSMC();
		void calc(state_type& X, double dt);
	};
#endif // !CONTROLLERS_H

