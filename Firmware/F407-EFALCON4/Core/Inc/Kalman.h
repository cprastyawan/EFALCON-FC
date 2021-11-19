#include <math.h>

typedef struct {
	double err_measure;
	double err_estimate;
	double q;
	double current_estimate;
	double last_estimate;
	double kalman_gain;
} Kalman_t;

void kalman_init(Kalman_t *kalman, double mea_e, double est_e, double q);
double kalman_updateEstimate(Kalman_t *kalman, double mea);
