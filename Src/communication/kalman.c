
#include "main.h"
#include "kalman.h"

kalfp kalman_update(kalman_filter_t *kalman,kalfp pre,kalfp get){
	kalfp k;
	
	kalman->v_pre+=kalman->v_noise_pre;
	k=kalman->v_pre/(kalman->v_pre+kalman->v_noise_get);
	kalman->now=pre+k*(get-pre);
	kalman->v_pre*=1-k;
	return kalman->now;
}


kalfp feiman_update(feiman_filter_t *feiman,kalfp pre1,kalfp pre2){
	kalfp k;
	feiman->v_1+=feiman->v_noise_1;
	feiman->v_2+=feiman->v_noise_2;
	feiman->now=(feiman->v_2*pre1+feiman->v_1*pre2)/(feiman->v_1+feiman->v_2);
	feiman->v_now=(feiman->v_1*feiman->v_2)/(feiman->v_1+feiman->v_2);
	
	return feiman->now;
}