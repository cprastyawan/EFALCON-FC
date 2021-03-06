#include "Kalman.h"

void kalman_init(Kalman_t *kalman, double mea_e, double est_e, double q){
	kalman->err_measure = mea_e;
	kalman->err_estimate = est_e;
	kalman->q = q;
}

double kalman_updateEstimate(Kalman_t *kalman, double mea){
	kalman->kalman_gain = kalman->err_estimate / (kalman->err_estimate + kalman->err_measure);
	kalman->current_estimate = kalman->last_estimate + kalman->kalman_gain * (mea - kalman->last_estimate);
	kalman->err_estimate = (1.0 - kalman->kalman_gain) * kalman->err_estimate + fabs(kalman->last_estimate - kalman->current_estimate) * kalman->q;
	kalman->last_estimate = kalman->current_estimate;

	return kalman->current_estimate;
}
