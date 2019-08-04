#ifndef KALMAN_H
#define KALMAN_H

typedef float kalfp;

typedef struct{
	kalfp v_pre;
	kalfp v_noise_get;
	kalfp v_noise_pre;
	kalfp now;
	
	
}kalman_filter_t;

typedef struct{
	kalfp v_noise_1;
	kalfp v_noise_2;
	kalfp v_1;
	kalfp v_2;
	kalfp now;
	kalfp v_now;
	
	
}feiman_filter_t;

kalfp kalman_update(kalman_filter_t *kalman,kalfp pre,kalfp get);
kalfp feiman_update(feiman_filter_t *feiman,kalfp pre1,kalfp pre2);

#endif
