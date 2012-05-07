/*
 * Stellarium
 * Copyright (C) 2003 Fabien Chereau
 *
 * This program is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License
 * as published by the Free Software Foundation; either version 2
 * of the License, or (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 51 Franklin Street, Suite 500, Boston, MA  02110-1335, USA.
 */

#include "StelObserver.hpp"
#include "StelUtils.hpp"
#include "SolarSystem.hpp"
#include "Planet.hpp"
#include "StelTranslator.hpp"
#include "StelApp.hpp"
#include "StelCore.hpp"

#include "StelLocationMgr.hpp"
#include "StelModuleMgr.hpp"

#include <QDebug>
#include <QSettings>
#include <QStringList>

double declination(float lat, float lon);

class ArtificialPlanet : public Planet
{
public:
	ArtificialPlanet(const PlanetP& orig);
	void setDest(const PlanetP& dest);
	void computeAverage(double f1);
private:
	void setRot(const Vec3d &r);
	static Vec3d getRot(const Planet* p);
	PlanetP dest;
	const QString orig_name;
	const QString orig_name_i18n;
};

ArtificialPlanet::ArtificialPlanet(const PlanetP& orig) :
		Planet("", 0, 0, 0, Vec3f(0,0,0), 0, "",
		NULL, NULL, 0, false, true, false), dest(0),
		orig_name(orig->getEnglishName()), orig_name_i18n(orig->getNameI18n())
{
	radius = 0;
	// set parent = sun:
	if (orig->getParent())
	{
		parent = orig->getParent();
		while (parent->getParent())
			parent = parent->getParent();
	}
	else
	{
		parent = orig; // sun
	}
	re = orig->getRotationElements();
	setRotEquatorialToVsop87(orig->getRotEquatorialToVsop87());
	setHeliocentricEclipticPos(orig->getHeliocentricEclipticPos());
}

void ArtificialPlanet::setDest(const PlanetP& dest)
{
	ArtificialPlanet::dest = dest;
	englishName = QString("%1->%2").arg(orig_name).arg(dest->getEnglishName());
	nameI18 = QString("%1->%2").arg(orig_name_i18n).arg(dest->getNameI18n());

	// rotation:
	const RotationElements &r(dest->getRotationElements());
	lastJD = StelApp::getInstance().getCore()->getJDay();

	re.offset = r.offset + fmod(re.offset - r.offset + 360.0*( (lastJD-re.epoch)/re.period - (lastJD-r.epoch)/r.period), 360.0);

	re.epoch = r.epoch;
	re.period = r.period;
	if (re.offset - r.offset < -180.f) re.offset += 360.f; else
	if (re.offset - r.offset >  180.f) re.offset -= 360.f;
}

void ArtificialPlanet::setRot(const Vec3d &r)
{
	const double ca = cos(r[0]);
	const double sa = sin(r[0]);
	const double cd = cos(r[1]);
	const double sd = sin(r[1]);
	const double cp = cos(r[2]);
	const double sp = sin(r[2]);
	Mat4d m;
	m.r[ 0] = cd*cp;
	m.r[ 4] = sd;
	m.r[ 8] = cd*sp;
	m.r[12] = 0;
	m.r[ 1] = -ca*sd*cp -sa*sp;
	m.r[ 5] =  ca*cd;
	m.r[ 9] = -ca*sd*sp +sa*cp;
	m.r[13] = 0;
	m.r[ 2] =  sa*sd*cp -ca*sp;
	m.r[ 6] = -sa*cd;
	m.r[10] =  sa*sd*sp +ca*cp;
	m.r[14] = 0;
	m.r[ 3] = 0;
	m.r[ 7] = 0;
	m.r[11] = 0;
	m.r[15] = 1.0;
	setRotEquatorialToVsop87(m);
}

Vec3d ArtificialPlanet::getRot(const Planet* p)
{
	const Mat4d m(p->getRotEquatorialToVsop87());
	const double cos_r1 = sqrt(m.r[0]*m.r[0]+m.r[8]*m.r[8]);
	Vec3d r;
	r[1] = atan2(m.r[4],cos_r1);
	// not well defined if cos(r[1])==0:
	if (cos_r1 <= 0.0)
	{
		// if (m.r[4]>0.0) sin,cos(a-p)=m.r[ 9],m.r[10]
		// else sin,cos(a+p)=m.r[ 9],m.r[10]
		// so lets say p=0:
		r[2] = 0.0;
		r[0] = atan2(m.r[9],m.r[10]);
	}
	else
	{
		r[0] = atan2(-m.r[6],m.r[5]);
		r[2] = atan2( m.r[8],m.r[0]);
	}
	return r;
}

void ArtificialPlanet::computeAverage(double f1)
{
	const double f2 = 1.0 - f1;
	// position
	setHeliocentricEclipticPos(getHeliocentricEclipticPos()*f1 + dest->getHeliocentricEclipticPos()*f2);

	// 3 Euler angles
	Vec3d a1(getRot(this));
	const Vec3d a2(getRot(dest.data()));
	if (a1[0]-a2[0] >  M_PI)
		a1[0] -= 2.0*M_PI;
	else
		if (a1[0]-a2[0] < -M_PI)
			a1[0] += 2.0*M_PI;
	if (a1[2]-a2[2] >  M_PI)
		a1[2] -= 2.0*M_PI;
	else
		if (a1[2]-a2[2] < -M_PI)
			a1[2] += 2.0*M_PI;
	setRot(a1*f1 + a2*f2);

	// rotation offset
	re.offset = f1*re.offset + f2*dest->getRotationElements().offset;
}




StelObserver::StelObserver(const StelLocation &loc) : currentLocation(loc)
{
	SolarSystem* ssystem = GETSTELMODULE(SolarSystem);
	planet = ssystem->searchByEnglishName(loc.planetName);
	if (planet==NULL)
	{
		qWarning() << "Can't create StelObserver on planet " + loc.planetName + " because it is unknown. Use Earth as default.";
		planet=ssystem->getEarth();
	}
	magneticDeclination = declination(loc.latitude,loc.longitude);
	qDebug() << "Declination: " << magneticDeclination;
}

StelObserver::~StelObserver()
{
}

const QSharedPointer<Planet> StelObserver::getHomePlanet(void) const
{
	return planet;
}

Vec3d StelObserver::getCenterVsop87Pos(void) const
{
	return getHomePlanet()->getHeliocentricEclipticPos();
}

double StelObserver::getDistanceFromCenter(void) const
{
	return getHomePlanet()->getRadius() + (currentLocation.altitude/(1000*AU));
}

Mat4d StelObserver::getRotAltAzToEquatorial(double jd) const
{
	double lat = currentLocation.latitude;
	// TODO: Figure out how to keep continuity in sky as reach poles
	// otherwise sky jumps in rotation when reach poles in equatorial mode
	// This is a kludge
	if( lat > 89.5 )  lat = 89.5;
	if( lat < -89.5 ) lat = -89.5;
	return Mat4d::zrotation((getHomePlanet()->getSiderealTime(jd)+currentLocation.longitude)*M_PI/180.)
		* Mat4d::yrotation((90.-lat)*M_PI/180.);
}

Mat4d StelObserver::getRotEquatorialToVsop87(void) const
{
	return getHomePlanet()->getRotEquatorialToVsop87();
}

SpaceShipObserver::SpaceShipObserver(const StelLocation& startLoc, const StelLocation& target, double atransitSeconds) : StelObserver(startLoc),
		moveStartLocation(startLoc), moveTargetLocation(target), artificialPlanet(NULL), transitSeconds(atransitSeconds)
{
	SolarSystem* ssystem = GETSTELMODULE(SolarSystem);
	PlanetP targetPlanet = ssystem->searchByEnglishName(moveTargetLocation.planetName);
	if (moveStartLocation.planetName!=moveTargetLocation.planetName)
	{
		PlanetP startPlanet = ssystem->searchByEnglishName(moveStartLocation.planetName);
		if (startPlanet.isNull() || targetPlanet.isNull())
		{
			qWarning() << "Can't move from planet " + moveStartLocation.planetName + " to planet " + moveTargetLocation.planetName + " because it is unknown";
			timeToGo = -1.;	// Will abort properly the move
			if (targetPlanet==NULL)
			{
				// Stay at the same position as a failover
				moveTargetLocation = moveStartLocation;
			}
			return;
		}

		ArtificialPlanet* artPlanet = new ArtificialPlanet(startPlanet);
		artPlanet->setDest(targetPlanet);
		artificialPlanet = QSharedPointer<Planet>(artPlanet);
	}
	planet = targetPlanet;
	timeToGo = transitSeconds;
}

SpaceShipObserver::~SpaceShipObserver()
{
	artificialPlanet.clear();
	planet.clear();
}

void SpaceShipObserver::update(double deltaTime)
{
	timeToGo -= deltaTime;

	// If move is over
	if (timeToGo <= 0.)
	{
		timeToGo = 0.;
		currentLocation = moveTargetLocation;
	}
	else
	{
		if (artificialPlanet)
		{
			// Update SpaceShip position
			static_cast<ArtificialPlanet*>(artificialPlanet.data())->computeAverage(timeToGo/(timeToGo + deltaTime));
			currentLocation.planetName = q_("SpaceShip");
			currentLocation.name = q_(moveStartLocation.planetName) + " -> " + q_(moveTargetLocation.planetName);
		}
		else
		{
			currentLocation.name = moveStartLocation.name + " -> " + moveTargetLocation.name;
			currentLocation.planetName = moveTargetLocation.planetName;
		}

		// Move the lon/lat/alt on the planet
		const double moveToMult = 1.-(timeToGo/transitSeconds);
		currentLocation.latitude = moveStartLocation.latitude - moveToMult*(moveStartLocation.latitude-moveTargetLocation.latitude);
		currentLocation.longitude = moveStartLocation.longitude - moveToMult*(moveStartLocation.longitude-moveTargetLocation.longitude);
		currentLocation.altitude = int(moveStartLocation.altitude - moveToMult*(moveStartLocation.altitude-moveTargetLocation.altitude));
	}
}

const QSharedPointer<Planet> SpaceShipObserver::getHomePlanet() const
{
	return (isObserverLifeOver() || artificialPlanet==NULL)  ? planet : artificialPlanet;
}


static short dec_tbl[17][37] = {\
1309,1184,1071,968,874,785,700,619,540,463,386,311,235,161,86,11,-65,-142,-222,-304,-388,-475,-564,-656,-750,-848,-949,1055,1168,1288,1417,1556,1704,1744,1591,1445,1309, \
853,774,711,657,609,563,516,466,412,354,293,230,167,106,47,-11,-69,-130,-197,-268,-345,-425,-506,-588,-669,-749,-830,-914,1004,1108,1243,1446,1781,1412,1129,962,853, \
466,455,442,429,417,405,389,367,333,288,232,168,103,42,-12,-57,-99,-144,-197,-260,-332,-407,-482,-551,-613,-667,-710,-740,-751,-727,-616,-252,224,404,458,471,466, \
303,306,304,301,299,299,298,293,275,240,185,113,36,-35,-91,-129,-155,-179,-212,-260,-324,-393,-458,-512,-551,-571,-567,-530,-449,-316,-150,9,131,213,263,291,303, \
219,224,226,225,224,223,225,226,217,190,136,57,-33,-113,-170,-203,-219,-226,-232,-252,-295,-351,-404,-441,-455,-443,-400,-324,-224,-120,-32,39,98,146,183,206,219, \
165,170,173,174,173,170,168,167,161,137,84,2,-91,-168,-218,-244,-255,-252,-231,-208,-210,-245,-288,-316,-318,-293,-243,-173,-97,-36,7,42,76,109,136,155,165, \
129,132,134,136,136,133,128,125,117,93,39,-42,-128,-196,-235,-249,-244,-221,-178,-126,-96,-104,-139,-172,-182,-168,-134,-86,-35,0,19,37,60,85,106,121,129, \
107,106,107,109,110,107,103,99,90,62,6,-70,-146,-202,-227,-223,-195,-155,-108,-62,-28,-20,-41,-71,-89,-88,-73,-43,-9,11,18,28,47,68,87,101,107, \
95,93,92,93,95,93,90,86,73,40,-16,-87,-152,-194,-204,-183,-142,-97,-58,-26,0,12,2,-21,-39,-44,-39,-23,-3,8,9,15,32,53,74,89,95, \
89,89,89,91,94,94,90,81,62,22,-35,-99,-152,-180,-177,-148,-104,-61,-29,-6,13,25,20,3,-13,-20,-20,-14,-5,-3,-7,-4,11,33,57,78,89, \
81,90,95,101,107,107,101,86,56,8,-53,-112,-152,-166,-153,-122,-81,-42,-13,6,21,32,30,17,4,-3,-6,-8,-9,-16,-27,-29,-18,5,34,62,81, \
69,90,106,118,127,129,120,97,56,-3,-70,-127,-157,-159,-139,-108,-70,-34,-5,14,28,38,40,32,23,16,9,1,-12,-31,-50,-58,-51,-29,3,38,69, \
53,88,116,138,152,154,142,112,59,-14,-91,-148,-172,-167,-143,-110,-72,-35,-4,19,35,49,56,57,53,45,33,14,-13,-46,-75,-90,-85,-62,-27,13,53, \
42,87,127,158,177,182,167,128,59,-34,-124,-184,-205,-196,-168,-131,-90,-48,-11,20,45,67,85,97,100,92,71,36,-11,-61,-102,-121,-116,-90,-51,-5,42, \
40,92,140,180,207,216,200,146,47,-81,-192,-254,-269,-252,-218,-173,-124,-74,-26,19,60,98,131,156,168,163,133,78,2,-74,-129,-151,-142,-112,-67,-15,40, \
43,102,158,207,243,258,237,153,-20,-220,-343,-383,-375,-340,-290,-232,-170,-106,-42,21,81,138,190,233,262,270,244,173,59,-61,-140,-169,-159,-124,-75,-17,43, \
64,126,185,237,274,273,175,-147,-490,-600,-603,-564,-506,-439,-365,-289,-211,-131,-52,27,104,178,249,315,371,414,434,415,330,172,10,-83,-107,-89,-48,5,64};


double declination(float lat, float lon)
{
	double decSW, decSE, decNW, decNE, decmin, decmax;
	double lonmin,latmin;
	short latmin_index,lonmin_index;
	/* set base point (latmin, lonmin) of grid */

	/* no limits test on lon */
	if (lon == 180) lonmin = 170;
	else lonmin = floor(lon/10)*10;

	/* supported lat's -60..60, so range check... */
	if (lat >= 80) latmin = 70;
	else if (lat < -80) latmin = -80;
	else latmin = floor(lat/10)*10;

	/* array index = (degrees+[60|180])/10 */
	latmin_index= (80+latmin)/10;
	lonmin_index= (180+lonmin)/10;

	decSW = dec_tbl[latmin_index][lonmin_index];
	decSE = dec_tbl[latmin_index][lonmin_index+1];
	decNE = dec_tbl[latmin_index+1][lonmin_index+1];
	decNW = dec_tbl[latmin_index+1][lonmin_index];

	/* approximate declination within the grid using bilinear interpolation */

	decmin = (lon - lonmin) / 10 * (decSE - decSW) + decSW;
	decmax = (lon - lonmin) / 10 * (decNE - decNW) + decNW;
	return   ((lat - latmin) / 10.f * (decmax - decmin) + decmin)/10.;
}

