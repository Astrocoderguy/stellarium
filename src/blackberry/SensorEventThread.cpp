/*
 * SensorEventThread.cpp
 *
 *  Created on: 2012-05-01
 *      Author: siarheig
 */

#include "SensorEventThread.h"
#include "SensorMgr.h"

#include <QDebug>

#include <bps/geolocation.h>
#include <bps/sensor.h>
#include <stdio.h>
#include <errno.h>
#include <math.h>

SensorEventThread::SensorEventThread(SensorMgr* sMgr):longitude(0.),latitude(100.),azimuth(0.f),pitch(0.f),_sMgr(sMgr) {

}

SensorEventThread::~SensorEventThread() {

}

void SensorEventThread::run() {
	qDebug() << "Running sensor thread...";
	bps_initialize();

    if (BPS_SUCCESS != geolocation_request_events(0)) {
    	qDebug() << "No access to GPS";
    	_sMgr->setGpsPermitted(false);
    	//qFatal("Error requesting geolocation events: %s", strerror(errno));
    }

    geolocation_set_period(30);

    if (BPS_SUCCESS != sensor_request_events(SENSOR_TYPE_AZIMUTH_PITCH_ROLL)) {
        qFatal("Error requesting gravity events: %s", strerror(errno));
    }
    sensor_set_rate(SENSOR_TYPE_AZIMUTH_PITCH_ROLL, 1000);

    while (true) {
        /*
         * Using a negative timeout (-1) in the call to bps_get_event(...)
         * ensures that we don't busy wait by blocking until an event is
         * available.
         */
        bps_event_t *event = NULL;
        bps_get_event(&event, -1);

        if (event) {
            /*
             * If it is a geolocation event, determine the response code and
             * handle the event accordingly.
             */
            if (bps_event_get_domain(event) == geolocation_get_domain()) {
                handle_geolocation_event(event);
            }
            else if (bps_event_get_domain(event) == sensor_get_domain()) {
            	handle_sensor_event(event);
            }
        }
    }

    /*
     * Stop geolocation events.
     */
    geolocation_stop_events(0);
    sensor_stop_events((sensor_type_t)0);

}

void
SensorEventThread::handle_sensor_event(bps_event_t *event)
{
    /* Double check that the event is valid */
    if (event == NULL || bps_event_get_code(event) != SENSOR_AZIMUTH_PITCH_ROLL_READING) {
        return;
    }
		float azimuth, pitch, roll;
		sensor_event_get_apr(event, &azimuth, &pitch, &roll);

		if(abs(this->azimuth - azimuth) > AZ_THRESHOLD ||
				abs(this->pitch - pitch) > PITCH_THRESHOLD) {
			this->azimuth = azimuth;
			this->pitch = pitch;
			//TODO notify
//			fprintf(stderr,"Az: %8.3f Pitch: %8.3f\n", this->azimuth, this->pitch);
			emit changed(longitude, latitude, azimuth, pitch);
		}
}

void
SensorEventThread::handle_geolocation_event(bps_event_t *event)
{

    double latitude = geolocation_event_get_latitude(event);
    double longitude = geolocation_event_get_longitude(event);
    //    double accuracy = geolocation_event_get_accuracy(event);
    //    double altitude = geolocation_event_get_altitude(event);
    //    bool altitude_valid = geolocation_event_is_altitude_valid(event);
    //    double altitude_accuracy = geolocation_event_get_altitude_accuracy(event);
    //    bool altitude_accuracy_valid = geolocation_event_is_altitude_accuracy_valid(event);
    //    double heading = geolocation_event_get_heading(event);
    //    bool heading_valid = geolocation_event_is_heading_valid(event);
    //    double speed = geolocation_event_get_speed(event);
    //    bool speed_valid = geolocation_event_is_speed_valid(event);
    //    double num_satellites_used = geolocation_event_get_num_satellites_used(event);
    //    bool num_satellites_valid = geolocation_event_is_num_satellites_valid(event);

//    if(abs(this->latitude - latitude) > GEO_THRESHOLD ||
//    		abs(this->longitude - longitude) > GEO_THRESHOLD) {
    	this->latitude = latitude;
    	this->longitude = longitude;
		//TODO notify
    	fprintf(stderr, "Lat: %8.3f Lon: %8.3f\n", this->latitude, this->longitude);
    	emit geo_changed(longitude, latitude);
//    }

}
