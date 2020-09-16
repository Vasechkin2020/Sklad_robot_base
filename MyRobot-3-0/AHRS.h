




//void Filter_Complementary()       //Простейший комплементарный фильтр
//{
//	//a(t) = (1-K) * (a(t-1) + gx*dt) + K * acc 
//	float coefficient = 0.07;
//	angleX = ((1 - coefficient) * (angleX + gyroX * intervalAccGyro)) + (coefficient * angleAccX);
//	angleY = ((1 - coefficient) * (angleY + gyroY * intervalAccGyro)) + (coefficient * angleAccY);
//	angleZ = angleGyroZ;
//}

		// Переменные для комплементарного фильтра
double X_angle_comp; // Переменная в которой храним предыдущее значение 
double Y_angle_comp; // Переменная в которой храним предыдущее значение 
double Z_angle_comp; // Переменная в которой храним предыдущее значение 

class AHRS {

private:
	/* Kalman filter variables */ //  Общие настройки для двух осей X и Y

	double Q_angle; // Process noise variance for the accelerometer
	double Q_bias; // Process noise variance for the gyro bias
	double R_measure; // Measurement noise variance - this is actually the variance of the measurement noise

	//--------------------------------------- ось X -----------------------------------------------
	double X_angle; // The angle calculated by the Kalman filter - part of the 2x1 state matrix
	double X_bias; // The gyro bias calculated by the Kalman filter - part of the 2x1 state matrix
	double X_rate; // Unbiased rate calculated from the rate and the calculated bias - you have to call getAngle to update the rate

	double X_P[2][2]; // Error covariance matrix - This is a 2x2 matrix

	//--------------------------------------- ось Y -----------------------------------------------
	double Y_angle; // The angle calculated by the Kalman filter - part of the 2x1 state matrix
	double Y_bias; // The gyro bias calculated by the Kalman filter - part of the 2x1 state matrix
	double Y_rate; // Unbiased rate calculated from the rate and the calculated bias - you have to call getAngle to update the rate

	double Y_P[2][2]; // Error covariance matrix - This is a 2x2 matrix


public:

	//double AngleX;         // Данные без фильтров
	//double AngleY;
	//double AngleZ;


	double compAngleX;		// Данные после комплементарного фильтра
	double compAngleY;
	double compAngleZ;


	double kalmAngleX;    // Данные после фильтра Калмана
	double kalmAngleY;
	double kalmAngleZ;


	double getCompAngleX(double comp_AngleX, double comp_GyroY, double dt)       //Простейший комплементарный фильтр
	{
		//a(t) = (1-K) * (a(t-1) + gx*dt) + K * acc 
		float coefficient = 0.05;
		X_angle_comp = ((1 - coefficient) * (X_angle_comp + comp_GyroY * dt)) + (coefficient * comp_AngleX);
		return X_angle_comp;
	}
	double getCompAngleY(double comp_AngleY, double comp_GyroX, double dt)       //Простейший комплементарный фильтр
	{
		//a(t) = (1-K) * (a(t-1) + gx*dt) + K * acc 
		float coefficient = 0.05;
		Y_angle_comp = ((1 - coefficient) * (Y_angle_comp + comp_GyroX * dt)) + (coefficient * comp_AngleY);
		return Y_angle_comp;
	}

	double getCompAngleZ(double magD, double comp_GyroZ, double dt)       //Простейший комплементарный фильтр
	{
		//a(t) = (1-K) * (a(t-1) + gx*dt) + K * acc 
		float coefficient = 0.001;
		//float magD = comp_AngleZ;
		float gyroD = Z_angle_comp + comp_GyroZ * dt;

		if (magD < 
			0) { magD = magD + 360; }
		if (gyroD < 180) { gyroD = gyroD + 360; }

		Z_angle_comp = ((1 - coefficient) * gyroD) + (coefficient * magD);

		if (Z_angle_comp >= 360) { Z_angle_comp = Z_angle_comp - 360; }

		return Z_angle_comp;
	}

	void Init_Kalman()
	{
		/* We will set the variables like so, these can also be tuned by the user */
		// Это настройки общие для двух осей X и Y
		Q_angle = 0.001f;
		Q_bias = 0.003f;
		R_measure = 0.03f;

		// Переменные для оси X
		X_angle = 0.0f; // Reset the angle
		X_bias = 0.0f; // Reset bias

		X_P[0][0] = 0.0f; // Since we assume that the bias is 0 and we know the starting angle (use setAngle), the error covariance matrix is set like so - see: http://en.wikipedia.org/wiki/Kalman_filter#Example_application.2C_technical
		X_P[0][1] = 0.0f;
		X_P[1][0] = 0.0f;
		X_P[1][1] = 0.0f;

		// Переменные для оси X

		Y_angle = 0.0f; // Reset the angle
		Y_bias = 0.0f; // Reset bias

		Y_P[0][0] = 0.0f; // Since we assume that the bias is 0 and we know the starting angle (use setAngle), the error covariance matrix is set like so - see: http://en.wikipedia.org/wiki/Kalman_filter#Example_application.2C_technical
		Y_P[0][1] = 0.0f;
		Y_P[1][0] = 0.0f;
		Y_P[1][1] = 0.0f;

		// Для комплементарного фильтра
		X_angle_comp = 0;
		Y_angle_comp = 0;
	};


	// The angle should be in degrees and  the rate should be in degrees per second  and the delta time in seconds

	double getkalmAngleX(double newAngleX, double newRateX, double dt)
	{
		// KasBot V2  -  Kalman filter module - http://www.x-firm.com/?page_id=145
			// Modified by Kristian Lauszus
			// See my blog post for more information: http://blog.tkjelectronics.dk/2012/09/a-practical-approach-to-kalman-filter-and-how-to-implement-it

			// Discrete Kalman filter time update equations - Time Update ("Predict")
			// Update xhat - Project the state ahead
			/* Step 1 */
		X_rate = newRateX - X_bias;
		X_angle += dt * X_rate;

		// Update estimation error covariance - Project the error covariance ahead
		/* Step 2 */
		X_P[0][0] += dt * (dt*X_P[1][1] - X_P[0][1] - X_P[1][0] + Q_angle);
		X_P[0][1] -= dt * X_P[1][1];
		X_P[1][0] -= dt * X_P[1][1];
		X_P[1][1] += Q_bias * dt;

		// Discrete Kalman filter measurement update equations - Measurement Update ("Correct")
		// Calculate Kalman gain - Compute the Kalman gain
		/* Step 4 */
		float S = X_P[0][0] + R_measure; // Estimate error
		/* Step 5 */
		float K[2]; // Kalman gain - This is a 2x1 vector
		K[0] = X_P[0][0] / S;
		K[1] = X_P[1][0] / S;

		// Calculate angle and bias - Update estimate with measurement zk (newAngle)
		/* Step 3 */
		float y = newAngleX - X_angle; // Angle difference
		/* Step 6 */
		X_angle += K[0] * y;
		X_bias += K[1] * y;

		// Calculate estimation error covariance - Update the error covariance
		/* Step 7 */
		float P00_temp = X_P[0][0];
		float P01_temp = X_P[0][1];

		X_P[0][0] -= K[0] * P00_temp;
		X_P[0][1] -= K[0] * P01_temp;
		X_P[1][0] -= K[1] * P00_temp;
		X_P[1][1] -= K[1] * P01_temp;

		return X_angle;
	};

	double getkalmAngleY(double newAngleY, double newRateY, double dt)
	{
		// KasBot V2  -  Kalman filter module - http://www.x-firm.com/?page_id=145
			// Modified by Kristian Lauszus
			// See my blog post for more information: http://blog.tkjelectronics.dk/2012/09/a-practical-approach-to-kalman-filter-and-how-to-implement-it

			// Discrete Kalman filter time update equations - Time Update ("Predict")
			// Update xhat - Project the state ahead
			/* Step 1 */
		Y_rate = newRateY - Y_bias;
		Y_angle += dt * Y_rate;

		// Update estimation error covariance - Project the error covariance ahead
		/* Step 2 */
		Y_P[0][0] += dt * (dt*Y_P[1][1] - Y_P[0][1] - Y_P[1][0] + Q_angle);
		Y_P[0][1] -= dt * Y_P[1][1];
		Y_P[1][0] -= dt * Y_P[1][1];
		Y_P[1][1] += Q_bias * dt;

		// Discrete Kalman filter measurement update equations - Measurement Update ("Correct")
		// Calculate Kalman gain - Compute the Kalman gain
		/* Step 4 */
		float S = Y_P[0][0] + R_measure; // Estimate error
		/* Step 5 */
		float K[2]; // Kalman gain - This is a 2x1 vector
		K[0] = Y_P[0][0] / S;
		K[1] = Y_P[1][0] / S;

		// Calculate angle and bias - Update estimate with measurement zk (newAngle)
		/* Step 3 */
		float y = newAngleY - Y_angle; // Angle difference
		/* Step 6 */
		Y_angle += K[0] * y;
		Y_bias += K[1] * y;

		// Calculate estimation error covariance - Update the error covariance
		/* Step 7 */
		float P00_temp = Y_P[0][0];
		float P01_temp = Y_P[0][1];

		Y_P[0][0] -= K[0] * P00_temp;
		Y_P[0][1] -= K[0] * P01_temp;
		Y_P[1][0] -= K[1] * P00_temp;
		Y_P[1][1] -= K[1] * P01_temp;

		return Y_angle;
	};


	void setAngleX(double newAngle) { X_angle = newAngle; }; // Used to set angle, this should be set as the starting angle
	void setAngleY(double newAngle) { Y_angle = newAngle; }; // Used to set angle, this should be set as the starting angle

	double getRateX() { return X_rate; }; // Return the unbiased rate X
	double getRateY() { return Y_rate; }; // Return the unbiased rate Y

	/* These are used to tune the Kalman filter */
	void setQangle(double newQ_angle) { Q_angle = newQ_angle; };
	void setQbias(double newQ_bias) { Q_bias = newQ_bias; };
	void setRmeasure(double newR_measure) { R_measure = newR_measure; };

	float getQangle() { return this->Q_angle; };
	float getQbias() { return this->Q_bias; };
	float getRmeasure() { return this->R_measure; };

};