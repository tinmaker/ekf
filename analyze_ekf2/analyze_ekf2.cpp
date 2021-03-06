
void Ekf::predictState()
{
	if (!_earth_rate_initialised) {
		if (_NED_origin_initialised) {
			calcEarthRateNED(_earth_rate_NED, _pos_ref.lat_rad);
			_earth_rate_initialised = true;
		}
	}

	// apply imu bias corrections
	Vector3f corrected_delta_ang = _imu_sample_delayed.delta_ang - _state.gyro_bias;
	Vector3f corrected_delta_vel = _imu_sample_delayed.delta_vel - _state.accel_bias;

	// correct delta angles for earth rotation rate
	corrected_delta_ang -= -_R_to_earth.transpose() * _earth_rate_NED * _imu_sample_delayed.delta_ang_dt;

	// convert the delta angle to a delta quaternion
	Quaternion dq;
	//将变化了的角度转为四元数
	dq.from_axis_angle(corrected_delta_ang);
	
	// rotate the previous quaternion by the delta quaternion using a quaternion multiplication
	//变化的四元数（角度）乘以以前的得到最新的角度四元数
	_state.quat_nominal = dq * _state.quat_nominal;

	// quaternions must be normalised whenever they are modified
	//四元数归一化
	_state.quat_nominal.normalize();

	// save the previous value of velocity so we can use trapzoidal integration
	Vector3f vel_last = _state.vel;

	// update transformation matrix from body to world frame
	//将有陀螺仪积分得到的四元数转为机体坐标系到世界坐标系的旋转矩阵
	_R_to_earth = quat_to_invrotmat(_state.quat_nominal);

	// calculate the increment in velocity using the current orientation
	//将机体坐标系下由加速度计得到的变化的速度转为世界坐标系
	_state.vel += _R_to_earth * corrected_delta_vel;

	// compensate for acceleration due to gravity
	//在世界坐标系下z轴加上重力导致的速度（加速度计z轴应该是指向天的）
	_state.vel(2) += _gravity_mss * _imu_sample_delayed.delta_vel_dt;
	
	// predict position states via trapezoidal integration of velocity
	//将最近两桢的速度求平均乘以时间得到位置
	_state.pos += (vel_last + _state.vel) * _imu_sample_delayed.delta_vel_dt * 0.5f;

	constrainStates();

	// calculate an average filter update time
	float input = 0.5f*(_imu_sample_delayed.delta_vel_dt + _imu_sample_delayed.delta_ang_dt);

	// filter and limit input between -50% and +100% of nominal value
	input = math::constrain(input,0.0005f * (float)(FILTER_UPDATE_PERIOD_MS),0.002f * (float)(FILTER_UPDATE_PERIOD_MS));
	_dt_ekf_avg = 0.99f * _dt_ekf_avg + 0.01f * input;
}


/*
 * Implement a strapdown INS algorithm using the latest IMU data at the current time horizon.
 * Buffer the INS states and calculate the difference with the EKF states at the delayed fusion time horizon.
 * Calculate delta angle, delta velocity and velocity corrections from the differences and apply them at the
 * current time horizon so that the INS states track the EKF states at the delayed fusion time horizon.
 * The inspiration for using a complementary filter to correct for time delays in the EKF
 * is based on the work by A Khosravian:
 * “Recursive Attitude Estimation in the Presence of Multi-rate and Multi-delay Vector Measurements”
 * A Khosravian, J Trumpf, R Mahony, T Hamel, Australian National University
*/
void Ekf::calculateOutputStates()
{
	// use latest IMU data
	imuSample imu_new = _imu_sample_new;

	// correct delta angles for bias offsets and scale factors
	Vector3f delta_angle;
	float dt_scale_correction = _dt_imu_avg/_dt_ekf_avg;
	delta_angle(0) = _imu_sample_new.delta_ang(0) - _state.gyro_bias(0)*dt_scale_correction;
	delta_angle(1) = _imu_sample_new.delta_ang(1) - _state.gyro_bias(1)*dt_scale_correction;
	delta_angle(2) = _imu_sample_new.delta_ang(2) - _state.gyro_bias(2)*dt_scale_correction;

	// correct delta velocity for bias offsets
	Vector3f delta_vel = _imu_sample_new.delta_vel - _state.accel_bias*dt_scale_correction;

	// Apply corrections to the delta angle required to track the quaternion states at the EKF fusion time horizon
	delta_angle += _delta_angle_corr;

	// convert the delta angle to an equivalent delta quaternions
	Quaternion dq;
	dq.from_axis_angle(delta_angle);

	// rotate the previous INS quaternion by the delta quaternions
	_output_new.time_us = imu_new.time_us;
	_output_new.quat_nominal = dq * _output_new.quat_nominal;

	// the quaternions must always be normalised afer modification
	_output_new.quat_nominal.normalize();

	// calculate the rotation matrix from body to earth frame
	_R_to_earth_now = quat_to_invrotmat(_output_new.quat_nominal);

	// rotate the delta velocity to earth frame
	Vector3f delta_vel_NED = _R_to_earth_now * delta_vel;

	// corrrect for measured accceleration due to gravity
	delta_vel_NED(2) += _gravity_mss * imu_new.delta_vel_dt;

	// save the previous velocity so we can use trapezidal integration
	Vector3f vel_last = _output_new.vel;

	// increment the INS velocity states by the measurement plus corrections
	_output_new.vel += delta_vel_NED;

	// use trapezoidal integration to calculate the INS position states
	_output_new.pos += (_output_new.vel + vel_last) * (imu_new.delta_vel_dt * 0.5f);

	// store INS states in a ring buffer that with the same length and time coordinates as the IMU data buffer
	if (_imu_updated) {
		_output_buffer.push(_output_new);
		_imu_updated = false;

		// get the oldest INS state data from the ring buffer
		// this data will be at the EKF fusion time horizon
		_output_sample_delayed = _output_buffer.get_oldest();

		// calculate the quaternion delta between the INS and EKF quaternions at the EKF fusion time horizon
		//_state.quat_nominal				是由_imu_sample_delayed的陀螺仪数据积分得到
		//它在后面融合过程时会被修正
		//_output_sample_delayed	是由_imu_sample_new的陀螺仪数据积分放到_output_buffer缓存中的
		//应用于输出互补滤波器，输出互补滤波用于将状态从融合时间范围推进到当前时间
		Quaternion quat_inv = _state.quat_nominal.inversed();
		Quaternion q_error =  _output_sample_delayed.quat_nominal * quat_inv;
		q_error.normalize();

		// convert the quaternion delta to a delta angle
		Vector3f delta_ang_error;
		float scalar;
		if (q_error(0) >= 0.0f) {
			scalar = -2.0f;

		} else {
			scalar = 2.0f;
		}
		delta_ang_error(0) = scalar * q_error(1);
		delta_ang_error(1) = scalar * q_error(2);
		delta_ang_error(2) = scalar * q_error(3);

		// calculate a gain that provides tight tracking of the estimator attitude states and
		// adjust for changes in time delay to maintain consistent damping ratio of ~0.7
		float time_delay = 1e-6f * (float)(_imu_sample_new.time_us - _imu_sample_delayed.time_us);
		time_delay = fmaxf(time_delay, _dt_imu_avg);
		float att_gain = 0.5f * _dt_imu_avg / time_delay;

		// calculate a corrrection to the delta angle
		// that will cause the INS to track the EKF quaternions
		_delta_angle_corr = delta_ang_error * att_gain;

		// calculate velocity and position tracking errors
		Vector3f vel_err = (_state.vel - _output_sample_delayed.vel);
		Vector3f pos_err = (_state.pos - _output_sample_delayed.pos);

		// collect magnitude tracking error for diagnostics
		_output_tracking_error[0] = delta_ang_error.norm();
		_output_tracking_error[1] = vel_err.norm();
		_output_tracking_error[2] = pos_err.norm();

		// calculate a velocity correction that will be applied to the output state history
		float vel_gain = _dt_ekf_avg / math::constrain(_params.vel_Tau, _dt_ekf_avg, 10.0f);
		_vel_err_integ += vel_err;
		Vector3f vel_correction = vel_err * vel_gain + _vel_err_integ * sq(vel_gain) * 0.1f;

		// calculate a position correction that will be applied to the output state history
		float pos_gain = _dt_ekf_avg / math::constrain(_params.pos_Tau, _dt_ekf_avg, 10.0f);
		_pos_err_integ += pos_err;
		Vector3f pos_correction = pos_err * pos_gain + _pos_err_integ * sq(pos_gain) * 0.1f;

		// loop through the output filter state history and apply the corrections to the velocity and position states
		// this method is too expensive to use for the attitude states due to the quaternion operations required
		// but does not introduce a time delay in the 'correction loop' and allows smaller tracking time constants
		// to be used
		outputSample output_states;
		unsigned max_index = _output_buffer.get_length() - 1;
		for (unsigned index=0; index <= max_index; index++) {
			output_states = _output_buffer.get_from_index(index);

			// a constant  velocity correction is applied
			output_states.vel += vel_correction;

			// a constant position correction is applied
			output_states.pos += pos_correction;

			// push the updated data to the buffer
			_output_buffer.push_to_index(index,output_states);

		}

		// update output state to corrected values
		_output_new = _output_buffer.get_newest();

	}
}

//setIMUData函数调用
bool Ekf::collect_imu(imuSample &imu)
{
	// accumulate and downsample IMU data across a period FILTER_UPDATE_PERIOD_MS long

	// copy imu data to local variables
	_imu_sample_new.delta_ang	= imu.delta_ang;
	_imu_sample_new.delta_vel	= imu.delta_vel;
	_imu_sample_new.delta_ang_dt	= imu.delta_ang_dt;
	_imu_sample_new.delta_vel_dt	= imu.delta_vel_dt;
	_imu_sample_new.time_us		= imu.time_us;

	// accumulate the time deltas
	_imu_down_sampled.delta_ang_dt += imu.delta_ang_dt;
	_imu_down_sampled.delta_vel_dt += imu.delta_vel_dt;

	// use a quaternion to accumulate delta angle data
	// this quaternion represents the rotation from the start to end of the accumulation period
	Quaternion delta_q;
	delta_q.rotate(imu.delta_ang);
	_q_down_sampled =  _q_down_sampled * delta_q;
	_q_down_sampled.normalize();

	// rotate the accumulated delta velocity data forward each time so it is always in the updated rotation frame
	matrix::Dcm<float> delta_R(delta_q.inversed());
	_imu_down_sampled.delta_vel = delta_R * _imu_down_sampled.delta_vel;

	// accumulate the most recent delta velocity data at the updated rotation frame
	// assume effective sample time is halfway between the previous and current rotation frame
	_imu_down_sampled.delta_vel += (_imu_sample_new.delta_vel + delta_R * _imu_sample_new.delta_vel) * 0.5f;

	// if the target time delta between filter prediction steps has been exceeded
	// write the accumulated IMU data to the ring buffer
	float target_dt = (float)(FILTER_UPDATE_PERIOD_MS) / 1000;
	if (_imu_down_sampled.delta_ang_dt >= target_dt - _last_dt_overrun) {

		// store the amount we have over-run the target update rate by
		_last_dt_overrun = _imu_down_sampled.delta_ang_dt - target_dt;

		imu.delta_ang     = _q_down_sampled.to_axis_angle();
		imu.delta_vel     = _imu_down_sampled.delta_vel;
		imu.delta_ang_dt  = _imu_down_sampled.delta_ang_dt;
		imu.delta_vel_dt  = _imu_down_sampled.delta_vel_dt;
		imu.time_us       = imu.time_us;

		_imu_down_sampled.delta_ang.setZero();
		_imu_down_sampled.delta_vel.setZero();
		_imu_down_sampled.delta_ang_dt = 0.0f;
		_imu_down_sampled.delta_vel_dt = 0.0f;
		_q_down_sampled(0) = 1.0f;
		_q_down_sampled(1) = _q_down_sampled(2) = _q_down_sampled(3) = 0.0f;
		return true;
	}

	return false;
}
