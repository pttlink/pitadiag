/* Diagnostic program for DMK Engineering LOX BeagleBoard Radio Interface 
 *
 * Copyright (c) 2016, Jim Dixon <jim@lambdatel.com>. All rights
 * reserved.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 * 
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 * 
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA  02111-1307 USA
 * 		 
 *
 * NOTE: Current version does not support USER I/O and PTT lines are non-inverted
 * which is likely to change in production board.
*/

#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <string.h>
#include <ctype.h>
#include <math.h>
#include <termios.h>
#include <sys/time.h>
#include <sys/ioctl.h>
#include <pthread.h>
#include <signal.h>
#include <sys/mman.h>
#include <alsa/asoundlib.h>
#include <linux/i2c-dev.h>		//Needed for I2C port
#include <math.h>


#define	GPIO_INPUTS 0x703f0
#define	MASK_GPIOS_OUTPUTS (MASK_GPIOS_GPIOS | 0xc00)
#define	GPIOIDLE (MASK_GPIOS_PTT0 | MASK_GPIOS_PTT1)

#define	IOEXP_IODIRA 0
#define	IOEXP_IODIRB 1
#define	IOEXP_GPIOA 0x12
#define	IOEXP_GPIOB 0x13
#define	IOEXP_OLATA 0x14
#define	IOEXP_OLATB 0x15

#define	MASK_GPIOS_PTT0 1
#define	MASK_GPIOS_PTT1 2
#define	MASK_GPIOS_COR0 4
#define	MASK_GPIOS_CTCSS0 8
#define	MASK_GPIOS_COR1 0x10
#define	MASK_GPIOS_CTCSS1 0x20
#define	MASK_GPIOS_GPIO1 0x40
#define	MASK_GPIOS_GPIO2 0x80
#define	MASK_GPIOS_GPIO3 0x100
#define	MASK_GPIOS_GPIO4 0x200
#define	MASK_GPIOS_GPIO5 0x400
#define	MASK_GPIOS_GPIO6 0x800
#define	MASK_GPIOS_GPIO7 0x1000
#define	MASK_GPIOS_GPIO8 0x2000
#define	MASK_GPIOS_GPIO9 0x4000
#define	MASK_GPIOS_GPIO10 0x8000
#define	MASK_GPIOS_GPIOS (MASK_GPIOS_GPIO1 | MASK_GPIOS_GPIO2 | MASK_GPIOS_GPIO3 | MASK_GPIOS_GPIO4 | MASK_GPIOS_GPIO5 |  \
	MASK_GPIOS_GPIO6 | MASK_GPIOS_GPIO7 | MASK_GPIOS_GPIO8 | MASK_GPIOS_GPIO9 | MASK_GPIOS_GPIO10)

#define	DEFAULT_IODIRA 0xfc  // Everyting input export PTT's
#define	DEFAULT_IODIRB 0xff  // Everyting input

#define IN  1
#define OUT 0

#define LOW  0
#define HIGH 1

#define	AUDIO_BLOCKSIZE 640
#define	AUDIO_SAMPLES_PER_BLOCK (AUDIO_BLOCKSIZE / 4)
#define	NFFT 128
#define	NFFTSQRT 7

#define	PLAYBACK_MAX 191

#define PASSBAND_LEVEL	354.8
#define	OTHER_LEVEL	10.0

#define	ADJFAC 0.97

struct tonevars
{
float	mycr;
float	myci;
} ;

void cdft(int, int, double *, int *, double *);

struct
{
	snd_pcm_t *icard, *ocard;
} alsa;

#if __BYTE_ORDER == __LITTLE_ENDIAN
static snd_pcm_format_t format = SND_PCM_FORMAT_S16_LE;
#else
static snd_pcm_format_t format = SND_PCM_FORMAT_S16_BE;
#endif

int readdev = -1;
int writedev = -1;
char *i2c_filename = "/dev/i2c-1";
int file_i2c = -1;
int i2c_addr = 0x27;
int rxmixerset = 500;
int txmixerset = 580;

float myfreq1 = 0.0,lev = 0.0,lev1 = 0.0,lev2 = 0.0;
int chnum = 0;

int shutdown = 0;

static unsigned long get_gpios(void)
{
int	x,y;

	x = i2c_smbus_read_byte_data(file_i2c,IOEXP_GPIOA);
	if (x < 0)
	{
		fprintf(stderr,"Failed to read IOEXP_GPIOA\n");
		exit(255);
	}
	y = i2c_smbus_read_byte_data(file_i2c,IOEXP_GPIOB);
	if (y < 0)
	{
		fprintf(stderr,"Failed to read IOEXP_GPIOB\n");
		exit(255);
	}
	return((y << 8) | x);
}

int set_gpios(int value)
{

	value ^= MASK_GPIOS_PTT0 | MASK_GPIOS_PTT1;

	if (i2c_smbus_write_byte_data(file_i2c,IOEXP_OLATB,value >> 8) < 0)
	{
		fprintf(stderr,"Failed to write IOEXP_OLATB\n");
		exit(255);
	}
	if (i2c_smbus_write_byte_data(file_i2c,IOEXP_OLATA,value & 0xff) < 0)
	{
		fprintf(stderr,"Failed to write IOEXP_OLATA\n");
		exit(255);
	}
	return 0;
}

void set_gpio_outmask(unsigned long mask)
{
	return;
}

static inline char *baboons(int v)
{
	if (v) return "1";
	return "0";
}

static int dioerror(unsigned long got, unsigned long should)
{
unsigned long err = got ^ should;
int	n = 0;

	if (err & MASK_GPIOS_COR0) {
		printf("Error on Port 1 COR (Port 2 PTT), got %s, should be %s\n",
			baboons(got & MASK_GPIOS_COR0),baboons(should & MASK_GPIOS_COR0));
		n++;
		}
	if (err & MASK_GPIOS_CTCSS0) {
		printf("Error on Port 1 CTCSS (Port 1 PTT), got %s, should be %s\n",
			baboons(got & MASK_GPIOS_CTCSS0),baboons(should & MASK_GPIOS_CTCSS0));
		n++;
		}
	if (err & MASK_GPIOS_COR1) {
		printf("Error on Port 2 COR (Port 1 PTT), got %s, should be %s\n",
			baboons(got & MASK_GPIOS_COR1),baboons(should & MASK_GPIOS_COR1));
		n++;
		}
	if (err & MASK_GPIOS_CTCSS1) {
		printf("Error on Port 2 CTCSS (Port 2 PTT), got %s, should be %s\n",
			baboons(got & MASK_GPIOS_CTCSS1),baboons(should & MASK_GPIOS_CTCSS1));
		n++;
		}
	return(n);
}

static int testio(unsigned long toout,unsigned long toexpect)
{
unsigned long c;

	set_gpios(toout);
	usleep(1000);
	c = get_gpios();
	return(dioerror(c,toexpect));
}

static int dgpioerror(unsigned long got, unsigned long should)
{
unsigned long err = got ^ should;
int	n = 0;

	if (err & MASK_GPIOS_GPIO1) {
		printf("Error on GPIO1, got %s, should be %s\n",
			baboons(got & MASK_GPIOS_GPIO1),baboons(should & MASK_GPIOS_GPIO1));
		n++;
		}
	if (err & MASK_GPIOS_GPIO2) {
		printf("Error on GPIO2, got %s, should be %s\n",
			baboons(got & MASK_GPIOS_GPIO2),baboons(should & MASK_GPIOS_GPIO2));
		n++;
		}
	if (err & MASK_GPIOS_GPIO3) {
		printf("Error on GPIO3, got %s, should be %s\n",
			baboons(got & MASK_GPIOS_GPIO3),baboons(should & MASK_GPIOS_GPIO3));
		n++;
		}
	if (err & MASK_GPIOS_GPIO4) {
		printf("Error on GPIO4, got %s, should be %s\n",
			baboons(got & MASK_GPIOS_GPIO4),baboons(should & MASK_GPIOS_GPIO4));
		n++;
		}
	if (err & MASK_GPIOS_GPIO5) {
		printf("Error on GPIO5, got %s, should be %s\n",
			baboons(got & MASK_GPIOS_GPIO5),baboons(should & MASK_GPIOS_GPIO5));
		n++;
		}
	return(n);
}

int testgpio(unsigned long toout,unsigned long toexpect)
{
unsigned long c;

	set_gpios(toout);
	usleep(1000);
	c = get_gpios();
	return(dgpioerror(c,toexpect));
}

static float get_tonesample(struct tonevars *tvars,float ddr,float ddi)
{

	float t;
	
	t =tvars->mycr*ddr-tvars->myci*ddi;
	tvars->myci=tvars->mycr*ddi+tvars->myci*ddr;
	tvars->mycr=t;
	
	t=2.0-(tvars->mycr*tvars->mycr+tvars->myci*tvars->myci);
	tvars->mycr*=t;
	tvars->myci*=t;
	return tvars->mycr;
}

static int outaudio(float freq)
{
short buf[AUDIO_SAMPLES_PER_BLOCK * 2];
float	f,ddr1,ddi1;
int	i,ch;
static struct tonevars t1;
snd_pcm_state_t state;

	ch = 0;
	if (chnum) ch = 1;
	if (freq > 0.0)
	{
		ddr1 = cos(freq*2.0*M_PI/8000.0);
		ddi1 = sin(freq*2.0*M_PI/8000.0);
	} else {
		t1.mycr = 1.0;
		t1.myci = 0.0;
	}
		
	memset(buf,0,sizeof(buf));
	for(i = 0; i < AUDIO_SAMPLES_PER_BLOCK * 2; i += 2)
	{
		if (freq > 0.0)
		{
			f = get_tonesample(&t1,ddr1,ddi1);
			buf[i + ch] = f * 32765;
		}
	}
	state = snd_pcm_state(alsa.ocard);
	if (state == SND_PCM_STATE_XRUN)
		snd_pcm_prepare(alsa.ocard);
	snd_pcm_writei(alsa.ocard, buf, AUDIO_SAMPLES_PER_BLOCK);
	return 0;
}

static int setamixer(int devnum,char *param, int v1, int v2)
{
int	type;
char	str[100];
snd_hctl_t *hctl;
snd_ctl_elem_id_t *id;
snd_ctl_elem_value_t *control;
snd_hctl_elem_t *elem;
snd_ctl_elem_info_t *info;

	sprintf(str,"hw:%d",devnum);
	if (snd_hctl_open(&hctl, str, 0)) return(-1);
	snd_hctl_load(hctl);
	snd_ctl_elem_id_alloca(&id);
	snd_ctl_elem_id_set_interface(id, SND_CTL_ELEM_IFACE_MIXER);
	snd_ctl_elem_id_set_name(id, param);  
	elem = snd_hctl_find_elem(hctl, id);
	if (!elem)
	{
		snd_hctl_close(hctl);
		fprintf(stderr,"Cannot find mixer element '%s'\n",param);
		return(-1);
	}
	snd_ctl_elem_info_alloca(&info);
	snd_hctl_elem_info(elem,info);
	type = snd_ctl_elem_info_get_type(info);
	snd_ctl_elem_value_alloca(&control);
	snd_ctl_elem_value_set_id(control, id);    
	switch(type)
	{
	    case SND_CTL_ELEM_TYPE_INTEGER:
	    case SND_CTL_ELEM_TYPE_BYTES:
	    case SND_CTL_ELEM_TYPE_ENUMERATED:
		snd_ctl_elem_value_set_integer(control, 0, v1);
		if (v2 >= 0) snd_ctl_elem_value_set_integer(control, 1, v2);
		break;
	    case SND_CTL_ELEM_TYPE_BOOLEAN:
		snd_ctl_elem_value_set_integer(control, 0, (v1 != 0));
		if (v2 >= 0) snd_ctl_elem_value_set_integer(control, 1, (v2 != 0));
		break;
	}
	if (snd_hctl_elem_write(elem, control))
	{
		snd_hctl_close(hctl);
		fprintf(stderr,"Cannot set value for mixer element '%s'\n",param);
		return(-1);
	}
	snd_hctl_close(hctl);
	return(0);
}

static int mixer_write(void)
{
int	v1,v2;

        if (setamixer(0,"HPOUT2L Input 1",0,-1) == -1) return -1;
        if (setamixer(0,"HPOUT2R Input 1",0,-1) == -1) return -1;
        if (setamixer(0,"HPOUT2L Input 2",0,-1) == -1) return -1;
        if (setamixer(0,"HPOUT2R Input 2",0,-1) == -1) return -1;
        if (setamixer(0,"HPOUT2L Input 1",15,-1) == -1) return -1;
        if (setamixer(0,"HPOUT2L Input 1 Volume",32,-1) == -1) return -1;
        if (setamixer(0,"HPOUT2R Input 1",16,-1) == -1) return -1;
        if (setamixer(0,"HPOUT2R Input 1 Volume",32,-1) == -1) return -1;
        if (setamixer(0,"HPOUT2 Digital Switch",1,1) == -1) return -1;
        if (setamixer(0,"IN3 High Performance Switch",1,-1) == -1) return -1;
        if (setamixer(0,"IN3L Digital Volume",102,-1) == -1) return -1;
        if (setamixer(0,"IN3R Digital Volume",102,-1) == -1) return -1;
        if (setamixer(0,"LHPF1 Input 1",11,-1) == -1) return -1;
        if (setamixer(0,"LHPF2 Input 1",12,-1) == -1) return -1;
        if (setamixer(0,"LHPF1 Mode",1,-1) == -1) return -1;
        if (setamixer(0,"LHPF2 Mode",1,-1) == -1) return -1;
        if (setamixer(0,"LHPF1 Coefficients",0x3f0,0) == -1) return -1;
        if (setamixer(0,"LHPF2 Coefficients",0x3f0,0) == -1) return -1;
        if (setamixer(0,"AIF1TX1 Input 1",47,-1) == -1) return -1;
        if (setamixer(0,"AIF1TX1 Input 1 Volume",32,-1) == -1) return -1;
        if (setamixer(0,"AIF1TX2 Input 1",48,-1) == -1) return -1;
        if (setamixer(0,"AIF1TX2 Input 1 Volume",32,-1) == -1) return -1;

	v1 =  txmixerset * PLAYBACK_MAX / 1000;
	v2 =  txmixerset * PLAYBACK_MAX / 1000;
        if (setamixer(0,"HPOUT2 Digital Volume",v1,v2) == -1) return -1;
	v1 =  rxmixerset * 32 / 1000;
	v2 =  rxmixerset * 32 / 1000;
	/* get interval step size */
	if (setamixer(0,"IN3L Volume",v1,-1) == -1) return -1;
	if (setamixer(0,"IN3R Volume",v2,-1) == -1) return -1;
	return 0;
}

static snd_pcm_t *alsa_card_init(char *dev, snd_pcm_stream_t stream)
{
	int err;
	snd_pcm_t *handle = NULL;
	snd_pcm_hw_params_t *hwparams = NULL;
	snd_pcm_sw_params_t *swparams = NULL;
	snd_pcm_uframes_t chunk_size = 0;
	unsigned period_time = 0;
	unsigned buffer_time = 80000;
	snd_pcm_uframes_t period_frames = 0;
	snd_pcm_uframes_t buffer_frames = 0;
	struct pollfd pfd;
	/* int period_bytes = 0; */
	snd_pcm_uframes_t buffer_size = 0;

	unsigned int rate = 8000;
	snd_pcm_uframes_t start_threshold, stop_threshold;

	err = snd_pcm_open(&handle, dev, stream, 0);
	if (err < 0) {
		fprintf(stderr, "snd_pcm_open failed: %s\n", snd_strerror(err));
		return NULL;
	}
	hwparams = alloca(snd_pcm_hw_params_sizeof());
	memset(hwparams, 0, snd_pcm_hw_params_sizeof());
	snd_pcm_hw_params_any(handle, hwparams);

	err = snd_pcm_hw_params_set_access(handle, hwparams, SND_PCM_ACCESS_RW_INTERLEAVED);
	if (err < 0)
	{
		fprintf(stderr, "set_access failed: %s\n", snd_strerror(err));
		return NULL;
	}
	err = snd_pcm_hw_params_set_format(handle, hwparams, format);
	if (err < 0) 
	{
		fprintf(stderr, "set_format failed: %s\n", snd_strerror(err));
		return NULL;
	}

	err = snd_pcm_hw_params_set_channels(handle, hwparams, 2);
	if (err < 0)
	{
		fprintf(stderr, "set_channels failed: %s\n", snd_strerror(err));
		return NULL;
	}
	err = snd_pcm_hw_params_set_rate_near(handle, hwparams, &rate, 0);
	if (err < 0)
	{
		fprintf(stderr, "set_rate failed: %s\n", snd_strerror(err));
		return NULL;
	}

	if (buffer_time == 0 && buffer_frames == 0) {
		err = snd_pcm_hw_params_get_buffer_time_max(hwparams,
							    &buffer_time, 0);
		assert(err >= 0);
		if (buffer_time > 500000)
			buffer_time = 500000;
	}
	if (period_time == 0 && period_frames == 0) {
		if (buffer_time > 0)
			period_time = buffer_time / 4;
		else
			period_frames = buffer_frames / 4;
	}
	if (period_time > 0)
		err = snd_pcm_hw_params_set_period_time_near(handle, hwparams,
							     &period_time, 0);
	else
		err = snd_pcm_hw_params_set_period_size_near(handle, hwparams,
							     &period_frames, 0);
	if (err < 0)
	{
		fprintf(stderr, "set_period time/size failed: %s\n", snd_strerror(err));
		return NULL;
	}

	if (buffer_time > 0) {
		err = snd_pcm_hw_params_set_buffer_time_near(handle, hwparams,
							     &buffer_time, 0);
	} else {
		err = snd_pcm_hw_params_set_buffer_size_near(handle, hwparams,
							     &buffer_frames);
	}

	if (err < 0)
	{
		fprintf(stderr, "set_buffer time/size failed: %s\n", snd_strerror(err));
		return NULL;
	}

	err = snd_pcm_hw_params(handle, hwparams);
	if (err < 0)
	{
		fprintf(stderr, "set_hw_params failed: %s\n", snd_strerror(err));
		return NULL;
	}

	snd_pcm_hw_params_get_period_size(hwparams, &chunk_size, 0);
	snd_pcm_hw_params_get_buffer_size(hwparams, &buffer_size);
	if (chunk_size == buffer_size) {
		fprintf(stderr,"Can't use period equal to buffer size (%lu == %lu)\n",
		      chunk_size, buffer_size);
		return NULL;
	}

	swparams = alloca(snd_pcm_sw_params_sizeof());
	memset(swparams, 0, snd_pcm_sw_params_sizeof());
	snd_pcm_sw_params_current(handle, swparams);

	err = snd_pcm_sw_params_set_avail_min(handle, swparams, chunk_size);

	if (err < 0)
	{
		fprintf(stderr, "set_avail_min failed: %s\n", snd_strerror(err));
		return NULL;
	}

	if (stream == SND_PCM_STREAM_PLAYBACK)
		start_threshold = buffer_size;
	else
		start_threshold = 1;

	err = snd_pcm_sw_params_set_start_threshold(handle, swparams, start_threshold);
	if (err < 0)
	{
		fprintf(stderr, "set_start_threshold failed: %s\n", snd_strerror(err));
		return NULL;
	}

	stop_threshold = buffer_size;
	err = snd_pcm_sw_params_set_stop_threshold(handle, swparams, stop_threshold);
	if (err < 0)
	{
		fprintf(stderr, "set_stopt_threshold failed: %s\n", snd_strerror(err));
		return NULL;
	}

	err = snd_pcm_sw_params(handle, swparams);
	if (err < 0)
	{
		fprintf(stderr, "set_sw_params failed: %s\n", snd_strerror(err));
		return NULL;
	}

	err = snd_pcm_poll_descriptors_count(handle);
	if (err <= 0)
	{
		fprintf(stderr, "Unable to get a poll descriptors count, error is %s\n", snd_strerror(err));
		return NULL;
	}
	if (err != 1)
	{
		fprintf(stderr, "Can't handle more than one device\n");
		return NULL;
	}
	snd_pcm_poll_descriptors(handle, &pfd, err);

	if (stream == SND_PCM_STREAM_CAPTURE)
		readdev = pfd.fd;
	else
		writedev = pfd.fd;

	return handle;
}

void *soundthread(void *this)
{
int	maxfd,ch;
snd_pcm_state_t state;

	while(!shutdown)
	{
		fd_set rfds,wfds;
		int res;
		char buf[AUDIO_BLOCKSIZE];
		float mylev,mylev1,mylev2;

		FD_ZERO(&rfds);
		FD_ZERO(&wfds);
		FD_SET(readdev,&rfds);
		FD_SET(writedev,&wfds);
		maxfd = readdev;
		if (writedev > maxfd) maxfd = writedev;
		res = select(maxfd + 1,&rfds,&wfds,NULL,NULL);
		if (!res) continue;
		if (res < 0)
		{
			perror("poll");
			exit(255);
		}
		if (FD_ISSET(writedev,&wfds))
		{
			outaudio(myfreq1);
			continue;
		}
		if (FD_ISSET(readdev,&rfds))
		{
			short *sbuf = (short *) buf;
			static double afft[(NFFT + 1) * 2 + 1],wfft[NFFT * 5 / 2];
			float buck;
			static int ipfft[NFFTSQRT + 2],i;

			state = snd_pcm_state(alsa.icard);
			if ((state != SND_PCM_STATE_PREPARED) && (state != SND_PCM_STATE_RUNNING)) {
				snd_pcm_prepare(alsa.icard);
			}
			res = snd_pcm_readi(alsa.icard, buf,AUDIO_SAMPLES_PER_BLOCK);
			if (res < AUDIO_SAMPLES_PER_BLOCK) {
				printf("Warining, short read!!\n");
				continue;
			}
			memset(afft,0,sizeof(double) * 2 * (NFFT + 1));
			for(i = 0; i < NFFT * 2; i += 2)
			{
				afft[i] = (double)(sbuf[i + (1 - chnum)] + 32768) / (double)68550.0;
			}
			ipfft[0] = 0;
			cdft(NFFT * 2,-1,afft,ipfft,wfft);
			mylev = 0.0;
			mylev1 = 0.0;
			mylev2 = 0.0;
			for(i = 1; i < NFFT / 2; i++)
			{
				float ftmp;

				ftmp = (afft[i * 2] * afft[i * 2]) + 
				    (afft[i * 2 + 1] * afft[i * 2 + 1]);

				mylev += ftmp;
				buck = (float) i * 62.50;
				if (myfreq1 > 0.0)
				{
					if (fabs(buck - myfreq1) < 1.5 * 46.875) mylev1 += ftmp;
				}
			}
			lev = (sqrt(mylev) / (float) (NFFT / 2)) * 4096.0;
			lev1 = (sqrt(mylev1) / (float) (NFFT / 2)) * 4096.0;
			ch = 0;
			if (chnum) ch = 1;
			memset(afft,0,sizeof(double) * 2 * (NFFT + 1));
			for(i = 0; i < NFFT * 2; i += 2)
			for(i = 0; i < NFFT * 2; i += 2)
			{
				afft[i] = (double)(sbuf[i + ch] + 32768) / (double)68550.0;
			}
			ipfft[0] = 0;
			cdft(NFFT * 2,-1,afft,ipfft,wfft);
			mylev2 = 0.0;
			for(i = 1; i < NFFT / 2; i++)
			{
				float ftmp;

				ftmp = (afft[i * 2] * afft[i * 2]) + 
				    (afft[i * 2 + 1] * afft[i * 2 + 1]);

				mylev2 += ftmp;
			}
			lev2 = (sqrt(mylev2) / (float) (NFFT / 2)) * 4096.0;
			lev *= ADJFAC;
			lev1 *= ADJFAC;
			lev2 *= ADJFAC;
		}
	}
	pthread_exit(NULL);
}

static int digital_test(void)
{
int	nerror = 0;

	printf("Testing digital I/O (PTT,COR and CTCSS)....\n");
	nerror += testio(MASK_GPIOS_PTT0 | MASK_GPIOS_PTT1,MASK_GPIOS_COR0 | MASK_GPIOS_CTCSS0 | MASK_GPIOS_COR1 | MASK_GPIOS_CTCSS1); /* NONE */
	nerror += testio(MASK_GPIOS_PTT1,MASK_GPIOS_COR0 | MASK_GPIOS_CTCSS1); /* Channel 1 PTT Asserted */
	nerror += testio(MASK_GPIOS_PTT0,MASK_GPIOS_COR1 | MASK_GPIOS_CTCSS0); /* Channel 2 PTT Asserted */
	nerror += testio(0,0);
	nerror += testio(MASK_GPIOS_PTT0 | MASK_GPIOS_PTT1,MASK_GPIOS_COR0 | MASK_GPIOS_CTCSS0 | MASK_GPIOS_COR1 | MASK_GPIOS_CTCSS1); /* NONE */
	if (!nerror) printf("Digital I/O passed!!\n");
	else printf("Digital I/O had %d errors!!\n",nerror);
	return(nerror);
}

static int gpio_test_a(void)
{
int	nerror = 0;

	printf("Not currently implemented!!\n");
#if 0
	printf("Testing GPIO loopback plug A....\n");
	set_gpio_outmask(0); /* Set to All input */
	nerror += testgpio(GPIOIDLE,MASK_GPIOS_GPIO2 | MASK_GPIOS_GPIO3); /* NONE (all input mode) */
	set_gpio_outmask(MASK_GPIOS_GPIO2 | MASK_GPIOS_GPIO3); /* GPIO2,3 outputs */
	nerror += testgpio(GPIOIDLE,MASK_GPIOS_GPIO1 | MASK_GPIOS_GPIO4 | MASK_GPIOS_GPIO5); /* NONE  */
	nerror += testgpio(GPIOIDLE | MASK_GPIOS_GPIO2,	MASK_GPIOS_GPIO2 | MASK_GPIOS_GPIO1 | MASK_GPIOS_GPIO5); /* GPIO2 */
	nerror += testgpio(GPIOIDLE | MASK_GPIOS_GPIO3, MASK_GPIOS_GPIO3 | MASK_GPIOS_GPIO4); /* GPIO3 */
	nerror += testgpio(GPIOIDLE | MASK_GPIOS_GPIO2 | MASK_GPIOS_GPIO3,MASK_GPIOS_GPIO2 | MASK_GPIOS_GPIO3); /* GPIO2 and GPIO3 */
	nerror += testgpio(GPIOIDLE,MASK_GPIOS_GPIO1 | MASK_GPIOS_GPIO4 | MASK_GPIOS_GPIO5); /* NONE  */
	set_gpio_outmask(0); /* Set to All input */
	nerror += testgpio(GPIOIDLE,MASK_GPIOS_GPIO2 | MASK_GPIOS_GPIO3); /* NONE (all input mode) */
	if (!nerror) printf("GPIO (with loopback plug A) passed!!\n");
	else printf("GPIO (with loopback plug A) had %d errors!!\n",nerror);
#endif
	return(nerror);
}

static int gpio_test_b(void)
{
int	nerror = 0;

	printf("Not currently implemented!!\n");
#if 0
	printf("Testing GPIO loopback plug B....\n");
	set_gpio_outmask(0); /* Set to All input */
	nerror += testgpio(GPIOIDLE,MASK_GPIOS_GPIO1 | MASK_GPIOS_GPIO4 | MASK_GPIOS_GPIO5); /* NONE (all input mode) */
	set_gpio_outmask(MASK_GPIOS_GPIO1 | MASK_GPIOS_GPIO4 | MASK_GPIOS_GPIO5); /* GPIO1,4,5 outputs */
	nerror += testgpio(GPIOIDLE,MASK_GPIOS_GPIO2 | MASK_GPIOS_GPIO3); /* NONE  */
	nerror += testgpio(GPIOIDLE | MASK_GPIOS_GPIO1,	MASK_GPIOS_GPIO1 | MASK_GPIOS_GPIO2); /* GPIO1 */
	nerror += testgpio(GPIOIDLE | MASK_GPIOS_GPIO5,	MASK_GPIOS_GPIO5 | MASK_GPIOS_GPIO2); /* GPIO5 */
	nerror += testgpio(GPIOIDLE | MASK_GPIOS_GPIO4, MASK_GPIOS_GPIO4 | MASK_GPIOS_GPIO3); /* GPIO4 */
	nerror += testgpio(GPIOIDLE | MASK_GPIOS_GPIO4 | MASK_GPIOS_GPIO5,MASK_GPIOS_GPIO4 | MASK_GPIOS_GPIO5); /* GPIO4 and GPIO5 */
	nerror += testgpio(GPIOIDLE,MASK_GPIOS_GPIO2 | MASK_GPIOS_GPIO3); /* NONE  */
	set_gpio_outmask(0); /* Set to All input */
	nerror += testgpio(GPIOIDLE,MASK_GPIOS_GPIO1 | MASK_GPIOS_GPIO4 | MASK_GPIOS_GPIO5); /* NONE (all input mode) */
	if (!nerror) printf("GPIO (with loopback plug B) passed!!\n");
	else printf("GPIO (with loopback plug B) had %d errors!!\n",nerror);
#endif
	return(nerror);
}

static int analog_test_one(float freq1,float dlev1,float dlev2,int v)
{
	int nerror = 0;
	myfreq1 = freq1;
	printf("Testing Analog at %1.f Hz...\n",freq1);
	usleep(1000000);
	if (fabs(lev1 - dlev1) > (dlev1 * 0.2))
	{
		printf("Main Channel Analog level for %.1f Hz (%.1f) is out of range!!\n",freq1,lev1);
		printf("Must be between %.1f and %.1f\n",dlev1 * .8, dlev1 * 1.2);
		nerror++;
	} else if (v) printf("Main Channel Level %.1f OK at %.1f Hz\n",lev1,freq1);
	if (lev2 > dlev2)
	{
		printf("Other Channel Analog level for %.1f Hz (%.1f) is out of range!!\n",freq1,lev2);
		printf("Must be less then %.1f\n",dlev2);
		nerror++;
	} else if (v) printf("Other channel Level %.1f OK at %.1f Hz\n",lev2,freq1);
	return(nerror);
}

static int analog_test(int v)
{
int	nerror = 0;

	chnum = 0;
	printf("For Channel 1 Tx, Channel 2 Rx:\n");
	nerror += analog_test_one(204.0,PASSBAND_LEVEL,OTHER_LEVEL,v);
	nerror += analog_test_one(1004.0,PASSBAND_LEVEL,OTHER_LEVEL,v);
	nerror += analog_test_one(2004.0,PASSBAND_LEVEL,OTHER_LEVEL,v);
	nerror += analog_test_one(3004.0,PASSBAND_LEVEL,OTHER_LEVEL,v);
	printf("For Channel 2 Tx, Channel 1 Rx:\n");
	chnum = 1;
	nerror += analog_test_one(204.0,PASSBAND_LEVEL,OTHER_LEVEL,v);
	nerror += analog_test_one(1004.0,PASSBAND_LEVEL,OTHER_LEVEL,v);
	nerror += analog_test_one(2004.0,PASSBAND_LEVEL,OTHER_LEVEL,v);
	nerror += analog_test_one(3004.0,PASSBAND_LEVEL,OTHER_LEVEL,v);
	if (!nerror) printf("Analog Test Passed!!\n");
	chnum = 0;
	return(nerror);
}

int main(int argc, char **argv)
{
int retval = 1;
char c;
pthread_t sthread;
pthread_attr_t attr;
struct termios t,t0;
float myfreq;

	printf("PITADiag, diagnostic program for the DMK Engineering PITA\n");
	printf("(Raspberry Pi 2/3) Radio Interface, version 0.1, 05/13/16\n\n");


	if ((file_i2c = open(i2c_filename, O_RDWR)) < 0)
	{
		fprintf(stderr,"Failed to open the i2c bus (addr %02X hex).\n",i2c_addr);
		exit(255);	
	}
	
	if (ioctl(file_i2c, I2C_SLAVE, i2c_addr) < 0)
	{
		fprintf(stderr,"Failed to acquire bus access and/or talk to slave (addr %02X hex).\n",i2c_addr);
		exit(255);	
	}
	
	if (i2c_smbus_write_byte_data(file_i2c,IOEXP_IODIRA,DEFAULT_IODIRA) < 0)
	{
		fprintf(stderr,"Failed to write to IO Expander (addr %02X hex).\n",i2c_addr);
		exit(255);	
	}

	if (i2c_smbus_write_byte_data(file_i2c,IOEXP_IODIRB,DEFAULT_IODIRB) < 0)
	{
		fprintf(stderr,"Failed to write to IO Expander (addr %02X hex).\n",i2c_addr);
		exit(255);	
	}


 	alsa.icard = alsa_card_init("hw:sndrpiwsp", SND_PCM_STREAM_CAPTURE);
	alsa.ocard = alsa_card_init("hw:sndrpiwsp", SND_PCM_STREAM_PLAYBACK);
	if (!alsa.icard || !alsa.ocard) {
		fprintf(stderr,"Problem opening alsa I/O devices\n");
		exit(255);
	}

	snd_pcm_prepare(alsa.icard);
	snd_pcm_start(alsa.icard);

	if (mixer_write() == -1) exit(255);

	set_gpios(GPIOIDLE);

	pthread_attr_init(&attr);
	pthread_attr_setdetachstate(&attr, PTHREAD_CREATE_DETACHED);
	pthread_create(&sthread,&attr,soundthread,NULL);

	usleep(500000);

	tcgetattr(fileno(stdin),&t0);
	for(;;)
	{
		char str[80];
		int errs = 0;

		tcsetattr(fileno(stdin),TCSANOW,&t0);
		myfreq = 0.0;
		myfreq1 = 0.0;
		printf("Menu:\r\n\n");
		printf("For Channel 1 Tx, Channel 2 Rx:\n");
		printf("1 - 1004Hz, 2 - 204Hz, 3 - 300Hz, 4 - 404Hz, 5 - 502Hz\n");
		printf("6 - 1502Hz, 7 - 2004Hz, 8 - 3004Hz\n");
		printf("For Channel 2 Tx, Channel 1 Rx:\n");
		printf("11 - 1004Hz, 22 - 204Hz, 33 - 300Hz, 44 - 404Hz, 55 - 502Hz\n");
		printf("66 - 1502Hz, 77 - 2004Hz, 88 - 3004Hz\n");
		printf("Tests, etc.\n");
		printf("t - test normal operation (use uppercase 'T' for verbose output)\n");
		printf("i - test digital RADIO signals only (COR,CTCSS,PTT)\n");
#if 0
		printf("a - test GPIO signals with loopback plug A\n");
		printf("b - test GPIO signals with loopback plug B\n");
#endif
		printf("c - show test (loopback) connector pinout\n");
		printf("q,x - exit program\r\n\n");
		printf("Enter your selection:");
		fflush(stdout);
		fgets(str,sizeof(str) - 1,stdin);
		c = str[0];
		if (isupper(c)) c = tolower(str[0]);
		switch (c)
		{
		    case 'x':
		    case 'q':
			goto exit;
		    case '1': 
			myfreq = 1004.0;
			break;
		    case '2': 
			myfreq = 204.0;
			break;
		    case '3': 
			myfreq = 300.0;
			break;
		    case '4': 
			myfreq = 404.0;
			break;
		    case '5': 
			myfreq = 502.0;
			break;
		    case '6': 
			myfreq = 1502.0;
			break;
		    case '7': 
			myfreq = 2004.0;
			break;
		    case '8': 
			myfreq = 3004.0;
			break;
		    case 0:
		    case '\n' :
		    case '\r' :
			myfreq = 0;
			break;
		    case 'i':
			digital_test();
			continue;
		    case 'a':
			gpio_test_a();
			continue;
		    case 'b':
			gpio_test_b();
			continue;
		    case 't':
		    case 'T':
			errs = digital_test();
			errs += analog_test(str[0] == 'T');
			if (!errs) printf("System Tests all Passed successfully!!!!\n");
			else printf("%d Error(s) found during test(s)!!!!\n",errs);
			printf("\n\n");
			continue;
		    case 'c': /* show test cable pinout */
			printf("Special Radio Test Cable Pinout:\n");
			printf("9 pin D-shell connector on each end\n");
			printf("  Port 1,Pin 1 to Port 2,Pin 1 (Gnd,Shield)\n");
			printf("  Port 1,Pins 2 & 3 to Port 2 Pin 7\n");
			printf("  Port 1,Pin 4 to Port 2,Pin 5\n");
			printf("  Port 1,Pin 5 to Port 2,Pin 4\n");
			printf("  Port 1,Pin 7 to Port 2,Pins 2 & 3\n\n");
#if 0
			printf("GPIO Looopback Plug A Pinout:\n");
			printf("15 pin D-shell connector\n");
			printf("  Pin 1 to Pin 5 & 11\n");
			printf("  Pin 4 to Pin 10\n\n");
			printf("GPIO Looopback Plug B Pinout:\n");
			printf("15 pin D-shell connector\n");
			printf("  Pin 2 to Pin 12\n");
			printf("  Pin 3 to Pin 9 & 13\n\n");
#endif
			continue;
		    default:
			continue;
		}
		myfreq1 = myfreq;
		chnum = 0;
		if ((strlen(str) > 1) && (str[0] == str[1])) chnum = 1;
		tcgetattr(fileno(stdin),&t);
		cfmakeraw(&t);
		t.c_cc[VTIME] = 10;
		t.c_cc[VMIN] = 0;
		tcsetattr(fileno(stdin),TCSANOW,&t);
		for(;;)
		{
			int c = getc(stdin);
			if (c > 0) break;
			usleep(500000);
			printf("Level at %.1f Hz: %.1f mV (RMS) %.1f mV (P-P)\r\n",myfreq,lev,lev * 2.828);
		}
	}
exit:
	shutdown = 1;
	pthread_join(sthread,NULL);
	set_gpios(GPIOIDLE);
	return retval;
}


