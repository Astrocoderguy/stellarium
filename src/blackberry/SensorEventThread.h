/*
 * SensorEventThread.h
 *
 *  Created on: 2012-05-01
 *      Author: siarheig
 */

#ifndef SENSOREVENTTHREAD_H_
#define SENSOREVENTTHREAD_H_

#include <qthread.h>
#include <bps/event.h>

static const float AZ_THRESHOLD = 1;
static const float PITCH_THRESHOLD = 1;

class SensorMgr;

class SensorEventThread: public QThread {
	Q_OBJECT
public:
	SensorEventThread(SensorMgr* sMgr);
	virtual ~SensorEventThread();

protected:
	virtual void run();

private:
	void handle_sensor_event(bps_event_t *event);
	void handle_geolocation_event(bps_event_t *event);

signals:
	void geo_changed(double lon, double lat);
	void changed(double lon, double lat, float az, float pi);


private:
	double longitude;
	double latitude;
	float  azimuth;
	float  pitch;
	SensorMgr* _sMgr;
};

#endif /* SENSOREVENTTHREAD_H_ */
