#ifndef MOTOR_H
#define MOTOR_H

#include <drivers/can.h>

#define BYTE(k, n)  (k >> n*8) & 0xFF

#define SDO_ID_TX 						0x600
#define SDO_ID_RX 						0x580

#define CS_DOWNLOAD 					0x40
#define CS_UPLOAD 						0x23

#define SDO_DEVICE_TYPE_INDEX 			0x1000
#define SDO_DEVICE_TYPE_SUBINDEX 		0x00

#define SDO_STATUS_REGISTER_INDEX 		0x3002
#define SDO_STATUS_REGISTER_SUBINDEX 	0x00

#define SDO_ERROR_REGISTER_INDEX 		0x3001
#define SDO_ERROR_REGISTER_SUBINDEX 	0X00

#define SDO_MODE_INDEX 					0x3003
#define SDO_MODE_SUBINDEX 				0x00

#define MOTOR_MODE_OFF 					0x00
#define MOTOR_MODE_CURRENT 				0x02
#define MOTOR_MODE_VEL 					0x03
#define MOTOR_MODE_SVEL 				0x05
#define MOTOR_MODE_POS 					0x07

#define SDO_ENABLE_INDEX 									0x3004
#define SDO_ENABLE_SUBINDEX 								0x00

#define SDO_SET_VELOCITY_INDEX 								0x3300
#define SDO_SET_VELOCITY_SUBINDEX 							0x00

#define SDO_SET_CURRENT_INDEX								0x3200
#define SDO_SET_CURRENT_SUBINDEX 							0x00

#define SDO_SET_SVELOCITY_INDEX								0x3500
#define SDO_SET_SVELOCITY_SUBINDEX							0x00

#define SDO_SET_POSITION_INDEX								0x3760
#define SDO_SET_POSITION_SUBINDEX							0x00

#define SDO_ACTUAL_CURRENT_INDEX							0x3262
#define SDO_ACTUAL_CURRENT_SUBINDEX							0x00

#define SDO_ACTUAL_VELOCITY_INDEX							0x3362
#define SDO_ACTUAL_VELOCITY_SUBINDEX						0x00

#define SDO_ACTUAL_POSITION_INDEX							0x3762
#define SDO_ACTUAL_POSITION_SUBINDEX						0x00

#define SDO_ACTUAL_POSFOLLOWINGERR_INDEX					0x3763
#define SDO_ACTUAL_POSFOLLOWINGERR_SUBINDEX					0x00

#define SDO_ACTUAL_CURRENT_LIMIT_POS_DIRECTION_INDEX		0x3221
#define SDO_ACTUAL_CURRENT_LIMIT_POS_DIRECTION_SUBINDEX		0x00

#define SDO_ACTUAL_CURRENT_LIMIT_NEG_DIRECTION_INDEX		0x3223
#define SDO_ACTUAL_CURRENT_LIMIT_NEG_DIRECTION_SUBINDEX		0x00

#define SDO_DYNAMIC_CURRENT_LIMIT_PEAK_CURRENT_INDEX		0x3224
#define SDO_DYNAMIC_CURRENT_LIMIT_PEAK_CURRENT_SUBINDEX		0x01

#define SDO_DYNAMIC_CURRENT_LIMIT_CONT_CURRENT_INDEX		0x3224
#define SDO_DYNAMIC_CURRENT_LIMIT_CONT_CURRENT_SUBINDEX		0x02

#define SDO_DYNAMIC_CURRENT_LIMIT_PEAK_TIME_INDEX			0x3224
#define SDO_DYNAMIC_CURRENT_LIMIT_PEAK_TIME_SUBINDEX		0x03

#define SDO_VELOCITY_LIMIT_POS_DIRECTION_INDEX				0x3321
#define SDO_VELOCITY_LIMIT_POS_DIRECTION_SUBINDEX			0x00

#define SDO_VELOCITY_LIMIT_NEG_DIRECTION_INDEX				0x3323
#define SDO_VELOCITY_LIMIT_NEG_DIRECTION_SUBINDEX			0x00

#define SDO_POSITION_LIMIT_MIN_INDEX						0x3720
#define SDO_POSITION_LIMIT_MIN_SUBINDEX						0x00

#define SDO_POSITION_LIMIT_MAX_INDEX						0x3720
#define SDO_POSITION_LIMIT_MAX_SUBINDEX						0x01

#define SDO_CLEAR_ERROR_INDEX								0x3000
#define SDO_CLEAR_ERROR_SUBINDEX							0x00

#define SDO_ENCODER_RESOLUTION_INDEX						0x3962
#define SDO_ENCODER_RESOLUTION_SUBINDEX						0x00

#define SDO_VELOCITY_FEEDBACK_INDEX							0x3350
#define SDO_VELOCITY_FEEDBACK_SUBINDEX						0x03

#define SDO_FACTOR_GROUP_INDEX								0x3B00
#define SDO_FACTOR_GROUP_SUBINDEX							0x00

#define SDO_RELATIVE_MOVING_INDEX							0x3791
#define SDO_RELATIVE_MOVING_SUBINDEX						0x00

#define SDO_ABSOLUTE_MOVING_INDEX							0x3790
#define SDO_ABSOLUTE_MOVING_SUBINDEX						0x00

#define SDO_SVEL_FEEDBACK_INDEX								0x3550
#define SDO_SVEL_FEEDBACK_SUBINDEX							0x00


struct motor_status{
	bool STAT_Enabled;
	bool STAT_Errror;
	bool STAT_Warning;
	bool STAT_Moving;
	bool STAT_Reached;
	bool STAT_Limit;
	bool STAT_FollowingError;
	bool STAT_HomingDone;
	bool STAT_Toggle;
	bool STAT_CmdToggle;
	bool STAT_CmdError;
	bool STAT_StopOrHalt;
	bool STAT_LimitCurrent;
	bool STAT_LimitVel;
	bool STAT_LimitPos;
	bool STAT_LimitSVel;
};

struct set_values {
	int32_t Current;
	int32_t Velocity;
	int32_t SVelocity;
	int32_t Position;
};

struct act_values{
	int32_t Current;
	int32_t Velocity;
	int32_t Position;
	int32_t PosFollowingErr;
};

struct act_limits{
	int32_t CurrentPosDirection;
	int32_t CurrentNegDirection;
	int32_t DynamicCurrentLimitPeakCurrent;
	int32_t DynamicCurrentLimitContCurrent;
	int32_t DynamicCurrentLimitPeakTime;
	int32_t VelocityPosDir;
	int32_t VelocityNegDir;
	int32_t PosMax;
	int32_t PosMin;
};

struct motor_dev {
	// Node ID from 1 to 127
	u8_t motor_id;
	// initialization variables
	bool initialized;
	bool init_error;
	// status register
	struct motor_status status_register;
	// error register
	u32_t error_register;
	//motor mode
	u8_t mode;

	struct set_values setpoints;

	struct act_values actual_values;

	struct act_limits actual_limits;
};

void zero_data(struct zcan_frame * );
void zero_table(u8_t * );
void int32_to_array(int32_t , u8_t * );
void int16_to_array(int16_t , u8_t * );

bool can_motor_request(struct device * dev, struct motor_dev * motor, u8_t frame_type, u16_t index, u8_t subindex, u8_t *data);

void motor_init(struct motor_dev * , u8_t , struct device * );

void get_motor_status(struct motor_dev * , struct device * );
void get_error_register(struct motor_dev * , struct device * );
void get_motor_mode(struct motor_dev * , struct device * );
void get_motor_setpoints(struct motor_dev *, struct device *);
void get_motor_actual_values(struct motor_dev * , struct device * );
void get_motor_actual_limits(struct motor_dev * , struct device * );

void motor_enable(struct motor_dev * , struct device * );
void motor_disable(struct motor_dev * , struct device * );
void motor_clear_error(struct motor_dev * , struct device * );

void motor_mode(struct motor_dev * , struct device *, u8_t );
void motor_set_factor_group(struct motor_dev * , struct device *, u8_t );
void motor_svel_feedback_encoder(struct motor_dev * , struct device *);
void motor_movr(struct motor_dev * , struct device *, int32_t );
void motor_mova(struct motor_dev * , struct device *, int32_t );


void motor_set_velocity(struct motor_dev * , struct device *, int32_t );
void motor_set_current(struct motor_dev * , struct device *, int32_t );
void motor_set_svelocity(struct motor_dev * , struct device *, int32_t );
void motor_set_position(struct motor_dev * , struct device *, int32_t );

void motor_set_actual_position(struct motor_dev * , struct device *, int32_t );
void motor_set_encoder_resolution(struct motor_dev * , struct device *, int32_t );

void motor_set_current_pos_direction_limit(struct motor_dev * motor, struct device *dev, int32_t val);
void motor_set_current_neg_direction_limit(struct motor_dev * motor, struct device *dev, int32_t val);
void motor_set_dynamic_peak_current_limit(struct motor_dev * motor, struct device *dev, int32_t val);
void motor_set_dynamic_cont_current_limit(struct motor_dev * motor, struct device *dev, int32_t val);
void motor_set_dynamic_peak_time_limit(struct motor_dev * motor, struct device *dev, int32_t val);
void motor_set_velocity_pos_direction_limit(struct motor_dev * motor, struct device *dev, int32_t val);
void motor_set_velocity_neg_direction_limit(struct motor_dev * motor, struct device *dev, int32_t val);

void motor_set_pos_max(struct motor_dev * motor, struct device *dev, int32_t val);
void motor_set_pos_min(struct motor_dev * motor, struct device *dev, int32_t val);


void zero_data(struct zcan_frame * frame){
	frame->data[4] = 0;
	frame->data[5] = 0;
	frame->data[6] = 0;
	frame->data[7] = 0;
}

void zero_table(u8_t * tb){
	tb[0] = 0;
	tb[1] = 0;
	tb[2] = 0;
	tb[3] = 0;
}
void int16_to_array(int16_t val, u8_t * array){
	u16_t u_val;
	if(val > 0) u_val = val;
	if(val < 0) u_val = 0xFFFF + (val + 1);

	array[0] = BYTE(u_val,0);
	array[1] = BYTE(u_val,1);
	array[2] = 0;
	array[3] = 0;
}

void int32_to_array(int32_t val, u8_t * array){
	u32_t u_val;
	if(val > 0) u_val = val;
	if(val < 0) u_val = 0xFFFFFFFF + (val + 1);

	array[0] = BYTE(u_val,0);
	array[1] = BYTE(u_val,1);
	array[2] = BYTE(u_val,2);
	array[3] = BYTE(u_val,3);
}


bool can_motor_request(struct device * dev, struct motor_dev * motor, u8_t frame_type, u16_t index, u8_t subindex, u8_t *data){
	if(!motor->initialized)
		return 0;

	struct fifo_can_frame *msg;

	struct zcan_frame frame = {
		.id_type = CAN_STANDARD_IDENTIFIER,
		.rtr = CAN_DATAFRAME,
		.std_id = SDO_ID_TX + motor->motor_id,
		.dlc = 8
	};
	frame.data[0] = frame_type;
	frame.data[1] = BYTE(index,0);
	frame.data[2] = BYTE(index,1);
	frame.data[3] = subindex;

	if(frame_type != CS_DOWNLOAD && frame_type != CS_UPLOAD)
		return 0;

	if(frame_type == CS_DOWNLOAD)
		zero_data(&frame);

	if(frame_type == CS_UPLOAD)
		for(int i = 0; i < 4; i++)
			frame.data[4+i] = data[i];

	can_send(dev, &frame, K_FOREVER, NULL, NULL);
	msg = k_fifo_get(&can_rx_fifo, K_FOREVER);

	//error handling - when we don't get a response to our request //

	if(SDO_ID_RX + motor->motor_id != msg->frame.std_id)
		return 0;

	if(msg->frame.data[1] != BYTE(index,0) && msg->frame.data[2] != BYTE(index,1) && msg->frame.data[3] != subindex)
		return 0;

	if(frame_type == CS_DOWNLOAD)
		for(int i = 0 ; i < 4; i++)
			data[i] = msg->frame.data[4+i];

	return 1;
}

void motor_init(struct motor_dev * motor, u8_t motor_id, struct device * dev){
	struct fifo_can_frame *msg;

	motor->initialized = false;
	motor->error_register = 0;
	motor->motor_id = motor_id;

	struct zcan_frame motor_init_frame = {
		.id_type = CAN_STANDARD_IDENTIFIER,
		.rtr = CAN_DATAFRAME,
		.std_id = SDO_ID_TX + motor_id,
		.dlc = 8
	};
	motor_init_frame.data[0] = CS_DOWNLOAD;
	motor_init_frame.data[1] = BYTE(SDO_DEVICE_TYPE_INDEX,0);
	motor_init_frame.data[2] = BYTE(SDO_DEVICE_TYPE_INDEX,1);
	motor_init_frame.data[3] = SDO_DEVICE_TYPE_SUBINDEX;

	while(!rxWork){ // waiting for can initialization
		k_sleep(100);
	}
	can_send(dev, &motor_init_frame, K_FOREVER, NULL, NULL);
	msg = k_fifo_get(&can_rx_fifo, K_FOREVER);

	if (msg->frame.std_id == SDO_ID_RX + motor_id && msg->frame.data[6] == 0x02){
		motor->initialized = true;
		motor->init_error = false;
	}

	get_motor_status(motor, dev);
	get_motor_mode(motor, dev);
	get_error_register(motor, dev);

}
void get_motor_status(struct motor_dev * motor, struct device * dev){
	if(motor->initialized){
		u8_t data[4];

		if(can_motor_request(dev, motor, CS_DOWNLOAD, SDO_STATUS_REGISTER_INDEX, SDO_STATUS_REGISTER_SUBINDEX, data)){
			motor->status_register.STAT_Enabled = (data[0] >> 0 ) & 0x1;
			motor->status_register.STAT_Errror = (data[0] >> 1 ) & 0x1;
			motor->status_register.STAT_Warning = (data[0] >> 2 ) & 0x1;
			motor->status_register.STAT_Moving = (data[0] >> 3 ) & 0x1;
			motor->status_register.STAT_Reached = (data[0] >> 4 ) & 0x1;
			motor->status_register.STAT_Limit = (data[0] >> 5 ) & 0x1;
			motor->status_register.STAT_FollowingError = (data[0] >> 6 ) & 0x1;
			motor->status_register.STAT_HomingDone = (data[0] >> 7 ) & 0x1;
			motor->status_register.STAT_Toggle = (data[1] >> 0 ) & 0x1;
			motor->status_register.STAT_CmdToggle = (data[1] >> 1 ) & 0x1;
			motor->status_register.STAT_CmdError = (data[1] >> 2 ) & 0x1;
			motor->status_register.STAT_StopOrHalt = (data[1] >> 3 ) & 0x1;
			motor->status_register.STAT_LimitCurrent = (data[1] >> 4 ) & 0x1;
			motor->status_register.STAT_LimitVel = (data[1] >> 5 ) & 0x1;
			motor->status_register.STAT_LimitPos = (data[1] >> 6 ) & 0x1;
			motor->status_register.STAT_LimitSVel = (data[1] >> 7 ) & 0x1;
		}
	}
}
void get_error_register(struct motor_dev * motor, struct device * dev){
	if(motor->initialized){
		u8_t data[4];
		if(can_motor_request(dev, motor, CS_DOWNLOAD, SDO_STATUS_REGISTER_INDEX, SDO_STATUS_REGISTER_SUBINDEX, data)){
			motor->error_register = data[0] & (data[1] << 8) & (data[2] << 16) & (data[3] << 24);
		}
	}
}

void get_motor_mode(struct motor_dev * motor, struct device * dev){
	u8_t data[4];
	if(can_motor_request(dev, motor, CS_DOWNLOAD, SDO_MODE_INDEX, SDO_MODE_SUBINDEX, data)){
		motor->mode = data[0];
	}
}

void get_motor_setpoints(struct motor_dev * motor, struct device * dev){
	u8_t data[4];
	if(can_motor_request(dev, motor, CS_DOWNLOAD, SDO_SET_VELOCITY_INDEX, SDO_SET_VELOCITY_SUBINDEX, data)){
		motor->setpoints.Velocity = data[0] | (data[1] << 8) | (data[2] << 16) | (data[3] << 24);
	}
	if(can_motor_request(dev, motor, CS_DOWNLOAD, SDO_SET_CURRENT_INDEX, SDO_SET_CURRENT_SUBINDEX, data)){
		motor->setpoints.Current = data[0] | (data[1] << 8) | (data[2] << 16) | (data[3] << 24);
	}
	if(can_motor_request(dev, motor, CS_DOWNLOAD, SDO_SET_SVELOCITY_INDEX, SDO_SET_SVELOCITY_SUBINDEX, data)){
		motor->setpoints.SVelocity = data[0] | (data[1] << 8) | (data[2] << 16) | (data[3] << 24);
	}
	if(can_motor_request(dev, motor, CS_DOWNLOAD, SDO_SET_POSITION_INDEX, SDO_SET_POSITION_SUBINDEX, data)){
		motor->setpoints.Position = data[0] | (data[1] << 8) | (data[2] << 16) | (data[3] << 24);
	}
}
void get_motor_actual_values(struct motor_dev * motor, struct device * dev){
	u8_t data[4];
	if(can_motor_request(dev, motor, CS_DOWNLOAD, SDO_ACTUAL_CURRENT_INDEX, SDO_ACTUAL_CURRENT_SUBINDEX, data)){
		motor->actual_values.Current = data[0] | (data[1] << 8) | (data[2] << 16) | (data[3] << 24);
	}
	if(can_motor_request(dev, motor, CS_DOWNLOAD, SDO_ACTUAL_VELOCITY_INDEX, SDO_ACTUAL_VELOCITY_SUBINDEX, data)){
		motor->actual_values.Velocity = data[0] | (data[1] << 8) | (data[2] << 16) | (data[3] << 24);
	}
	if(can_motor_request(dev, motor, CS_DOWNLOAD, SDO_ACTUAL_POSFOLLOWINGERR_INDEX, SDO_ACTUAL_POSFOLLOWINGERR_SUBINDEX, data)){
		motor->actual_values.PosFollowingErr = data[0] | (data[1] << 8) | (data[2] << 16) | (data[3] << 24);
	}
	if(can_motor_request(dev, motor, CS_DOWNLOAD, SDO_ACTUAL_POSITION_INDEX, SDO_ACTUAL_POSITION_SUBINDEX, data)){
		motor->actual_values.Position = data[0] | (data[1] << 8) | (data[2] << 16) | (data[3] << 24);
	}
}


void get_motor_actual_limits(struct motor_dev * motor, struct device * dev){
	u8_t data[4];
	if(can_motor_request(dev, motor, CS_DOWNLOAD, SDO_ACTUAL_CURRENT_LIMIT_POS_DIRECTION_INDEX, SDO_ACTUAL_CURRENT_LIMIT_POS_DIRECTION_SUBINDEX, data)){
		motor->actual_limits.CurrentPosDirection = data[0] | (data[1] << 8) | (data[2] << 16) | (data[3] << 24);
	}
	if(can_motor_request(dev, motor, CS_DOWNLOAD, SDO_ACTUAL_CURRENT_LIMIT_NEG_DIRECTION_INDEX, SDO_ACTUAL_CURRENT_LIMIT_NEG_DIRECTION_SUBINDEX, data)){
		motor->actual_limits.CurrentNegDirection = data[0] | (data[1] << 8) | (data[2] << 16) | (data[3] << 24);
	}
	if(can_motor_request(dev, motor, CS_DOWNLOAD, SDO_DYNAMIC_CURRENT_LIMIT_PEAK_CURRENT_INDEX, SDO_DYNAMIC_CURRENT_LIMIT_PEAK_CURRENT_SUBINDEX, data)){
		motor->actual_limits.DynamicCurrentLimitPeakCurrent = data[0] | (data[1] << 8) | (data[2] << 16) | (data[3] << 24);
	}
	if(can_motor_request(dev, motor, CS_DOWNLOAD, SDO_DYNAMIC_CURRENT_LIMIT_CONT_CURRENT_INDEX, SDO_DYNAMIC_CURRENT_LIMIT_CONT_CURRENT_SUBINDEX, data)){
		motor->actual_limits.DynamicCurrentLimitContCurrent = data[0] | (data[1] << 8) | (data[2] << 16) | (data[3] << 24);
	}
	if(can_motor_request(dev, motor, CS_DOWNLOAD, SDO_DYNAMIC_CURRENT_LIMIT_PEAK_TIME_INDEX, SDO_DYNAMIC_CURRENT_LIMIT_PEAK_TIME_SUBINDEX, data)){
		motor->actual_limits.DynamicCurrentLimitPeakTime = data[0] | (data[1] << 8) | (data[2] << 16) | (data[3] << 24);
	}
	if(can_motor_request(dev, motor, CS_DOWNLOAD, SDO_VELOCITY_LIMIT_POS_DIRECTION_INDEX, SDO_VELOCITY_LIMIT_POS_DIRECTION_SUBINDEX, data)){
		motor->actual_limits.VelocityPosDir = data[0] | (data[1] << 8) | (data[2] << 16) | (data[3] << 24);
	}
	if(can_motor_request(dev, motor, CS_DOWNLOAD, SDO_VELOCITY_LIMIT_NEG_DIRECTION_INDEX, SDO_VELOCITY_LIMIT_NEG_DIRECTION_SUBINDEX, data)){
		motor->actual_limits.VelocityNegDir = data[0] | (data[1] << 8) | (data[2] << 16) | (data[3] << 24);
	}
	if(can_motor_request(dev, motor, CS_DOWNLOAD, SDO_POSITION_LIMIT_MIN_INDEX, SDO_POSITION_LIMIT_MIN_SUBINDEX, data)){
		motor->actual_limits.PosMin = data[0] | (data[1] << 8) | (data[2] << 16) | (data[3] << 24);
	}
	if(can_motor_request(dev, motor, CS_DOWNLOAD, SDO_POSITION_LIMIT_MAX_INDEX, SDO_POSITION_LIMIT_MAX_SUBINDEX, data)){
		motor->actual_limits.PosMax = data[0] | (data[1] << 8) | (data[2] << 16) | (data[3] << 24);
	}
}

void motor_enable(struct motor_dev * motor, struct device * dev){
	if(motor->initialized){

		u8_t data[4];
		zero_table(data);
		data[0] = 1;

		if(can_motor_request(dev, motor, CS_UPLOAD, SDO_ENABLE_INDEX, SDO_ENABLE_SUBINDEX, data)){
			motor->error_register = data[0] & (data[1] << 8) & (data[2] << 16) & (data[3] << 24);
		}

		get_motor_status(motor, dev);
	}
}


void motor_disable(struct motor_dev * motor, struct device * dev){
	u8_t data[4];
	zero_table(data);

	if(can_motor_request(dev, motor, CS_UPLOAD, SDO_ENABLE_INDEX, SDO_ENABLE_SUBINDEX, data)){
		motor->error_register = data[0] & (data[1] << 8) & (data[2] << 16) & (data[3] << 24);
	}

	get_motor_status(motor, dev);
}
void motor_clear_error(struct motor_dev * motor, struct device * dev){

	u8_t data[4];
	zero_table(data);

	if(can_motor_request(dev, motor, CS_UPLOAD, SDO_CLEAR_ERROR_INDEX, SDO_CLEAR_ERROR_SUBINDEX, data)){

	}

	get_motor_status(motor, dev);
}


void motor_svel_feedback_encoder(struct motor_dev * motor, struct device * dev){
	u8_t data[4];
	zero_table(data);

	if(can_motor_request(dev, motor, CS_UPLOAD, SDO_SVEL_FEEDBACK_INDEX, SDO_SVEL_FEEDBACK_SUBINDEX, data)){

	}

}
void motor_mode(struct motor_dev * motor, struct device *dev, u8_t mode){

	u8_t data[4];
	zero_table(data);
	data[0] = mode;
	if(can_motor_request(dev, motor, CS_UPLOAD, SDO_MODE_INDEX, SDO_MODE_SUBINDEX, data)){

	}
	get_motor_status(motor, dev);
}

void motor_set_factor_group(struct motor_dev * motor, struct device *dev, u8_t mode){

	u8_t data[4];
	zero_table(data);
	data[0] = mode;
	if(can_motor_request(dev, motor, CS_UPLOAD, SDO_FACTOR_GROUP_INDEX, SDO_FACTOR_GROUP_SUBINDEX, data)){

	}
}

void motor_set_velocity(struct motor_dev * motor, struct device *dev, int32_t velocity){
	u8_t data[4];
	int32_to_array(velocity, data);

	if(can_motor_request(dev, motor, CS_UPLOAD, SDO_SET_VELOCITY_INDEX, SDO_SET_VELOCITY_SUBINDEX, data)){
		motor->setpoints.Velocity = velocity;
	}
}
void motor_set_current(struct motor_dev * motor, struct device *dev, int32_t current){
	u8_t data[4];
	int32_to_array(current, data);

	if(can_motor_request(dev, motor, CS_UPLOAD, SDO_SET_CURRENT_INDEX, SDO_SET_CURRENT_SUBINDEX, data)){
		motor->setpoints.Current = current;
	}
}
void motor_set_svelocity(struct motor_dev * motor, struct device *dev, int32_t svelocity){
	u8_t data[4];
	int32_to_array(svelocity, data);

	if(can_motor_request(dev, motor, CS_UPLOAD, SDO_SET_SVELOCITY_INDEX, SDO_SET_SVELOCITY_SUBINDEX, data)){
		motor->setpoints.SVelocity = svelocity;
	}
}
void motor_set_position(struct motor_dev * motor, struct device *dev, int32_t position){
	u8_t data[4];
	int32_to_array(position, data);

	if(can_motor_request(dev, motor, CS_UPLOAD, SDO_SET_POSITION_INDEX, SDO_SET_POSITION_SUBINDEX, data)){
		motor->setpoints.Position = position;
	}
}

void motor_set_current_pos_direction_limit(struct motor_dev * motor, struct device *dev, int32_t val){
	u8_t data[4];
	int32_to_array(val, data);

	if(can_motor_request(dev, motor, CS_UPLOAD, SDO_ACTUAL_CURRENT_LIMIT_POS_DIRECTION_INDEX, SDO_ACTUAL_CURRENT_LIMIT_POS_DIRECTION_SUBINDEX, data)){
		motor->actual_limits.CurrentPosDirection = val;
	}
}
void motor_set_current_neg_direction_limit(struct motor_dev * motor, struct device *dev, int32_t val){
	u8_t data[4];
	int32_to_array(val, data);

	if(can_motor_request(dev, motor, CS_UPLOAD, SDO_ACTUAL_CURRENT_LIMIT_NEG_DIRECTION_INDEX, SDO_ACTUAL_CURRENT_LIMIT_NEG_DIRECTION_SUBINDEX, data)){
		motor->actual_limits.CurrentNegDirection = val;
	}
}
void motor_set_dynamic_peak_current_limit(struct motor_dev * motor, struct device *dev, int32_t val){
	u8_t data[4];
	int32_to_array(val, data);

	if(can_motor_request(dev, motor, CS_UPLOAD, SDO_DYNAMIC_CURRENT_LIMIT_PEAK_CURRENT_INDEX, SDO_DYNAMIC_CURRENT_LIMIT_PEAK_CURRENT_SUBINDEX, data)){
		motor->actual_limits.DynamicCurrentLimitPeakCurrent = val;
	}
}
void motor_set_dynamic_cont_current_limit(struct motor_dev * motor, struct device *dev, int32_t val){
	u8_t data[4];
	int32_to_array(val, data);

	if(can_motor_request(dev, motor, CS_UPLOAD, SDO_DYNAMIC_CURRENT_LIMIT_CONT_CURRENT_INDEX, SDO_DYNAMIC_CURRENT_LIMIT_CONT_CURRENT_SUBINDEX, data)){
		motor->actual_limits.DynamicCurrentLimitContCurrent = val;
	}
}
void motor_set_dynamic_peak_time_limit(struct motor_dev * motor, struct device *dev, int32_t val){
	u8_t data[4];
	int32_to_array(val, data);

	if(can_motor_request(dev, motor, CS_UPLOAD, SDO_DYNAMIC_CURRENT_LIMIT_PEAK_TIME_INDEX, SDO_DYNAMIC_CURRENT_LIMIT_PEAK_TIME_SUBINDEX, data)){
		motor->actual_limits.DynamicCurrentLimitPeakTime = val;
	}
}
void motor_set_velocity_pos_direction_limit(struct motor_dev * motor, struct device *dev, int32_t val){
	u8_t data[4];
	int32_to_array(val, data);

	if(can_motor_request(dev, motor, CS_UPLOAD, SDO_VELOCITY_LIMIT_POS_DIRECTION_INDEX, SDO_VELOCITY_LIMIT_POS_DIRECTION_SUBINDEX, data)){
		motor->actual_limits.VelocityPosDir = val;
	}
}
void motor_set_velocity_neg_direction_limit(struct motor_dev * motor, struct device *dev, int32_t val){
	u8_t data[4];
	int32_to_array(val, data);

	if(can_motor_request(dev, motor, CS_UPLOAD, SDO_VELOCITY_LIMIT_NEG_DIRECTION_INDEX, SDO_VELOCITY_LIMIT_NEG_DIRECTION_SUBINDEX, data)){
		motor->actual_limits.VelocityNegDir = val;
	}
}

void motor_set_pos_max(struct motor_dev * motor, struct device *dev, int32_t val){
	u8_t data[4];
	int32_to_array(val, data);

	if(can_motor_request(dev, motor, CS_UPLOAD, SDO_POSITION_LIMIT_MAX_INDEX, SDO_POSITION_LIMIT_MAX_SUBINDEX, data)){
		motor->actual_limits.PosMin = val;
	}
}
void motor_set_pos_min(struct motor_dev * motor, struct device *dev, int32_t val){
	u8_t data[4];
	int32_to_array(val, data);

	if(can_motor_request(dev, motor, CS_UPLOAD, SDO_POSITION_LIMIT_MIN_INDEX, SDO_POSITION_LIMIT_MIN_SUBINDEX, data)){
		motor->actual_limits.PosMin = val;
	}
}

void motor_set_encoder_resolution(struct motor_dev * motor, struct device *dev, int32_t val){
	u8_t data[4];
	int32_to_array(val, data);

	if(can_motor_request(dev, motor, CS_UPLOAD, SDO_ENCODER_RESOLUTION_INDEX, SDO_ENCODER_RESOLUTION_SUBINDEX, data)){
		motor->actual_limits.PosMin = val;
	}
}



void motor_set_actual_position(struct motor_dev * motor, struct device *dev, int32_t val){
	u8_t data[4];
	int32_to_array(val, data);

	if(can_motor_request(dev, motor, CS_UPLOAD, SDO_ACTUAL_POSITION_INDEX, SDO_ACTUAL_POSITION_SUBINDEX, data)){
		motor->actual_values.Position = val;
	}
}


void motor_movr(struct motor_dev * motor, struct device *dev, int32_t val){
	u8_t data[4];
	int32_to_array(val, data);

	if(can_motor_request(dev, motor, CS_UPLOAD, SDO_RELATIVE_MOVING_INDEX, SDO_RELATIVE_MOVING_SUBINDEX, data)){
	}
}

void motor_mova(struct motor_dev * motor, struct device *dev, int32_t val){
	u8_t data[4];
	int32_to_array(val, data);

	if(can_motor_request(dev, motor, CS_UPLOAD, SDO_ABSOLUTE_MOVING_INDEX, SDO_ABSOLUTE_MOVING_SUBINDEX, data)){

	}
}

#endif /* MOTOR_H */
