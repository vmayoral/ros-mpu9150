/*
 * =====================================================================================
 *
 *       Filename:  mpu9150_node.cpp
 *
 *    Description:  
 *
 *        Version:  1.0
 *        Created:  27/07/13 15:06:50
 *       Revision:  none
 *       Compiler:  gcc
 *
 *         Author:  Víctor Mayoral Vilches <v.mayoralv@gmail.com>
 *   Organization:  
 *
 * =====================================================================================
 */
#include "ros/ros.h"
#include "std_msgs/String.h"
#include <sstream>

#include <stdio.h>
#include <stdint.h>
#include <stdlib.h>
#include <string.h>
#include <signal.h>
#include <getopt.h>
#include <errno.h>

#include "mpu9150.h"
#include "linux_glue.h"
#include "local_defaults.h"

int set_cal(int mag, char *cal_file);
void read_loop(unsigned int sample_rate);
void print_fused_euler_angles(mpudata_t *mpu);
void print_fused_quaternion(mpudata_t *mpu);
void print_calibrated_accel(mpudata_t *mpu);
void print_calibrated_mag(mpudata_t *mpu);
void register_sig_handler();
void sigint_handler(int sig);

int done;

int main(int argc, char **argv)
{
  ros::init(argc, argv, "mpu9150");

  ros::NodeHandle n;

  ros::Publisher chatter_pub = n.advertise<std_msgs::String>("mpu9150_topic", 1000);
  ros::Rate loop_rate(10);

  /* Init the sensor */
    int opt, len;
	int i2c_bus = DEFAULT_I2C_BUS;
	int sample_rate = DEFAULT_SAMPLE_RATE_HZ;
	int yaw_mix_factor = DEFAULT_YAW_MIX_FACTOR;
	int verbose = 0;
	char *mag_cal_file = NULL;
	char *accel_cal_file = NULL;
	unsigned long loop_delay;
	mpudata_t mpu;

	register_sig_handler();

	mpu9150_set_debug(verbose);

	if (mpu9150_init(i2c_bus, sample_rate, yaw_mix_factor))
		exit(1);

	set_cal(0, accel_cal_file);
	set_cal(1, mag_cal_file);

	if (accel_cal_file)
		free(accel_cal_file);

	if (mag_cal_file)
		free(mag_cal_file);

	memset(&mpu, 0, sizeof(mpudata_t));

	if (sample_rate == 0)
		return -1;

	loop_delay = (1000 / sample_rate) - 2;

	printf("\nEntering MPU read loop (ctrl-c to exit)\n\n");

	linux_delay_ms(loop_delay);

  /**
   * A count of how many messages we have sent. This is used to create
   * a unique string for each message.
   */
  int count = 0;
  while (ros::ok())
  {
    /*
    std_msgs::String msg;
    std::stringstream ss;

    ss << "hello world " << count;
    msg.data = ss.str();

    ROS_INFO("%s", msg.data.c_str());
    */
    
    std_msgs::String msg;
    std::stringstream ss;

	if (mpu9150_read(&mpu) == 0) {
		print_fused_euler_angles(&mpu);
	    //ss << "\rX: %0.0f Y: %0.0f Z: %0.0f        ",
	    ss << "\rX: " << mpu.fusedEuler[VEC3_X] * RAD_TO_DEGREE <<
            " Y: " << mpu.fusedEuler[VEC3_Y] * RAD_TO_DEGREE <<
			" Z: " << mpu.fusedEuler[VEC3_Z] * RAD_TO_DEGREE << count;

		// printf_fused_quaternions(&mpu);
	    // print_calibrated_accel(&mpu);
	    // print_calibrated_mag(&mpu);

       msg.data = ss.str();
       ROS_INFO("%s", msg.data.c_str());
	}

	linux_delay_ms(loop_delay);

    chatter_pub.publish(msg);

    ros::spinOnce();

    loop_rate.sleep();
    ++count;
  }
  return 0;
}

void print_fused_euler_angles(mpudata_t *mpu)
{
	printf("\rX: %0.0f Y: %0.0f Z: %0.0f        ",
			mpu->fusedEuler[VEC3_X] * RAD_TO_DEGREE, 
			mpu->fusedEuler[VEC3_Y] * RAD_TO_DEGREE, 
			mpu->fusedEuler[VEC3_Z] * RAD_TO_DEGREE);

	fflush(stdout);
}

void print_fused_quaternions(mpudata_t *mpu)
{
	printf("\rW: %0.2f X: %0.2f Y: %0.2f Z: %0.2f        ",
			mpu->fusedQuat[QUAT_W],
			mpu->fusedQuat[QUAT_X],
			mpu->fusedQuat[QUAT_Y],
			mpu->fusedQuat[QUAT_Z]);

	fflush(stdout);
}

void print_calibrated_accel(mpudata_t *mpu)
{
	printf("\rX: %05d Y: %05d Z: %05d        ",
			mpu->calibratedAccel[VEC3_X], 
			mpu->calibratedAccel[VEC3_Y], 
			mpu->calibratedAccel[VEC3_Z]);

	fflush(stdout);
}

void print_calibrated_mag(mpudata_t *mpu)
{
	printf("\rX: %03d Y: %03d Z: %03d        ",
			mpu->calibratedMag[VEC3_X], 
			mpu->calibratedMag[VEC3_Y], 
			mpu->calibratedMag[VEC3_Z]);

	fflush(stdout);
}

int set_cal(int mag, char *cal_file)
{
	int i;
	FILE *f;
	char buff[32];
	long val[6];
	caldata_t cal;

	if (cal_file) {
		f = fopen(cal_file, "r");
		
		if (!f) {
			perror("open(<cal-file>)");
			return -1;
		}
	}
	else {
		if (mag) {
			f = fopen("./magcal.txt", "r");
		
			if (!f) {
				printf("Default magcal.txt not found\n");
				return 0;
			}
		}
		else {
			f = fopen("./accelcal.txt", "r");
		
			if (!f) {
				printf("Default accelcal.txt not found\n");
				return 0;
			}
		}		
	}

	memset(buff, 0, sizeof(buff));
	
	for (i = 0; i < 6; i++) {
		if (!fgets(buff, 20, f)) {
			printf("Not enough lines in calibration file\n");
			break;
		}

		val[i] = atoi(buff);

		if (val[i] == 0) {
			printf("Invalid cal value: %s\n", buff);
			break;
		}
	}

	fclose(f);

	if (i != 6) 
		return -1;

	cal.offset[0] = (short)((val[0] + val[1]) / 2);
	cal.offset[1] = (short)((val[2] + val[3]) / 2);
	cal.offset[2] = (short)((val[4] + val[5]) / 2);

	cal.range[0] = (short)(val[1] - cal.offset[0]);
	cal.range[1] = (short)(val[3] - cal.offset[1]);
	cal.range[2] = (short)(val[5] - cal.offset[2]);
	
	if (mag) 
		mpu9150_set_mag_cal(&cal);
	else 
		mpu9150_set_accel_cal(&cal);

	return 0;
}

void register_sig_handler()
{
	struct sigaction sia;

	bzero(&sia, sizeof sia);
	sia.sa_handler = sigint_handler;

	if (sigaction(SIGINT, &sia, NULL) < 0) {
		perror("sigaction(SIGINT)");
		exit(1);
	} 
}

void sigint_handler(int sig)
{
	done = 1;
}
