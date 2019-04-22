// Base classes for NMEA parsing

class CCapabilities {
private:
	unsigned m_GPRMC:1;
	unsigned m_GPGGA:1;
public:
	inline CCapabilities(): m_GPRMC(0), m_GPGGA(0) { }
	inline void SetGPRMC() { m_GPRMC=1; }
	inline void SetGPGGA() { m_GPGGA=1; }
	inline bool HasGPRMC() const { return (m_GPRMC!=0); }
	inline bool HasGPGGA() const { return (m_GPGGA!=0); }
};

// GPRMC nmea sentence
class CGPRMC {
private:
	unsigned int	m_Time;
	bool		m_Validity;
	float		m_Latitude;
	float		m_Longitude;
	float		m_Speed;
	float		m_Azimuth;
public:
	inline CGPRMC();
	// getters
	inline unsigned int	Time() const;
	inline bool			Validity() const;
	inline float		Latitude() const;
	inline float		Longitude() const;
	inline float		Speed() const;
	inline float		Azimuth() const;
	// setters
	inline void		SetTime(unsigned int);
	inline void		SetValidity(bool);
	inline void		SetLatitude(float);
	inline void		SetLongitude(float);
	inline void		SetSpeed(float);
	inline void		SetAzimuth(float);
	inline void		Reset();
	// queryers
	inline bool		HasTime() const;
	inline bool		HasValidity() const;
	inline bool		HasLatitude() const;
	inline bool		HasLongitude() const;
	inline bool		HasSpeed() const;
	inline bool		HasAzimuth() const;
	inline bool		Empty() const;
private:
	inline CGPRMC(const CGPRMC&) { }
	inline CGPRMC& operator=(const CGPRMC&) { return *this; }
};

// GPGGA nmea sentence
class CGPGGA {
private:
	unsigned int	m_Time;
	float		m_HDOP;
	int		m_Satellites;
public:
	inline CGPGGA();
	// getters
	inline unsigned int	Time() const;
	inline float		HDOP() const;
	inline int		Satellites() const;
	// setters
	inline void		SetTime(unsigned int);
	inline void		SetHDOP(float);
	inline void		SetSatellites(int);
	inline void		Reset();
	// queryers
	inline bool		HasTime() const;
	inline bool		HasHDOP() const;
	inline bool		HasSatellites() const;
	inline bool		Empty() const;
private:
	inline CGPGGA(const CGPGGA&) { }
	inline CGPGGA& operator=(const CGPGGA&) { return *this; }
};

// GPRMC
CGPRMC::CGPRMC() { Reset(); }
inline unsigned int	CGPRMC::Time() const		{	return m_Time; }
inline bool		CGPRMC::Validity() const	{	return m_Validity; }
inline float		CGPRMC::Latitude() const	{	return m_Latitude; }
inline float		CGPRMC::Longitude() const	{	return m_Longitude; }
inline float		CGPRMC::Speed() const		{	return m_Speed; }
inline float		CGPRMC::Azimuth() const		{	return m_Azimuth; }
inline void		CGPRMC::SetTime(unsigned int v)	{	m_Time=v; }
inline void		CGPRMC::SetValidity(bool v)	{	m_Validity=v; }
inline void		CGPRMC::SetLatitude(float v)	{	m_Latitude=v; }
inline void		CGPRMC::SetLongitude(float v)	{	m_Longitude=v; }
inline void		CGPRMC::SetSpeed(float v)	{	m_Speed=v; }
inline void		CGPRMC::SetAzimuth(float v)	{	m_Azimuth=v; }
inline void		CGPRMC::Reset() 		{
	m_Time		= 0xffffffff;
	m_Validity	= false;
	m_Latitude	= 1000.f;
	m_Longitude	= 1000.f;
	m_Speed		= -1000.f;
	m_Azimuth	= 1000.f;
}
inline bool		CGPRMC::HasTime() const		{	return (m_Time!=0xffffffff); }
inline bool		CGPRMC::HasValidity() const	{	return true; }
inline bool		CGPRMC::HasLatitude() const	{	return (m_Latitude!=1000.f); }
inline bool		CGPRMC::HasLongitude() const	{	return (m_Longitude!=1000.f); }
inline bool		CGPRMC::HasSpeed() const	{	return (m_Speed!=-1000.f); }
inline bool		CGPRMC::HasAzimuth() const	{	return (m_Azimuth!=1000.f); }
inline bool		CGPRMC::Empty() const		{
	return (!HasTime()) && (!HasLatitude()) && (!HasLongitude()) && (!HasSpeed()) && (!HasAzimuth());
}

// GPGGA
CGPGGA::CGPGGA()
{ Reset(); }
inline unsigned int	CGPGGA::Time() const		{	return m_Time; }
inline float CGPGGA::HDOP() const			{	return m_HDOP; }
inline int  CGPGGA::Satellites() const			{	return m_Satellites; }
inline void	CGPGGA::SetTime(unsigned int v)		{	m_Time=v; }
inline void	CGPGGA::SetHDOP(float v)		{	m_HDOP=v; }
inline void CGPGGA::SetSatellites(int v)		{	m_Satellites=v; }
inline void CGPGGA::Reset() {
	m_Time		= 0xffffffff;
	m_HDOP		= -1000.f;
	m_Satellites= -1;
}
inline bool	CGPGGA::HasTime() const		{	return (m_Time!=0xffffffff); }
inline bool CGPGGA::HasHDOP() const		{	return (m_HDOP!=-1000.f); }
inline bool CGPGGA::HasSatellites() const	{	return (m_Satellites!=-1); }
inline bool CGPGGA::Empty() const	{
	return (!HasTime()) && (!HasHDOP()) && (!HasSatellites());
}


// Main program


#include <iostream>
#include <string>
#include <fstream>
#include <sstream>
#include <vector>
#include <list>
#include <cmath>

CCapabilities caps;
CGPRMC	lastGPRMC;
CGPGGA	lastGPGGA;

// Convert NMEA angles to decimal degrees
static void convertDegrees(float& f) {
	int degrees = (int)(f/100.0f);
	float minutes = f - degrees*100.0f;
	minutes = ((minutes*100.0f)/6000.0f);
	f = degrees + minutes;
}

static void convertKnotsToMetersPerSecond(float& v) {
v*=.5144444444444444f;
}

// (longitude,latitude) <-> meters (spherical projection)

static const float degreeArcLength	= 111226.29991434248924368723f;
static const float degreeArcLengthRec	 = .00000899067936962857f;
void convertWGS84ToSI(float fLongitude, float fLatitude, float& fCoordX, float& fCoordY) {
	if (fabsf(fLatitude)>85.f)
		fLatitude = (fLatitude<.0f)?-85.f:85.f;
	fCoordY = fLatitude*degreeArcLength;
	fCoordX = fLongitude*degreeArcLength;
	fCoordX*=cosf(fLatitude*.01745329251994329576f);
}

void convertSIToWGS84(float fCoordX, float fCoordY, float& fLongitude, float& fLatitude) {
	fLatitude=fCoordY*degreeArcLengthRec;
	fLongitude=fCoordX*degreeArcLengthRec;
	fLongitude/=cosf(fLatitude*.01745329251994329576f);
}

struct SPosition {
	SPosition(): longitude(.0f), latitude(.0f), azimuth(.0f), speed(.0f), dop(.0f), timestamp(0), validity(false) { }
	SPosition(const SPosition& o): longitude(o.longitude), latitude(o.latitude), azimuth(o.azimuth), speed(o.speed), dop(o.dop), timestamp(o.timestamp), validity(o.validity) { }
	float longitude;
	float latitude;
	float azimuth;
	float speed;
	float dop;
	unsigned int timestamp;
	bool validity;
};

int main(int argc, char** argv) {
	const char* filename = "nmea.txt";
	const char* outfile="nmea.kml";

	std::cerr << "Converting file \"" << filename << "\"" << std::endl;

	std::vector<SPosition> positions;
	std::cout.precision(6);
	std::cout.precision(20);
	std::ifstream ifs(filename);
	if (!ifs.is_open()) return 1;
	char line[512];

	std::vector<std::string> fields;
	std::string currentFrame;

	while (ifs.good()) {
		ifs.getline(line,512, '\n');
		std::string sline(line);
		size_t trim=0;
		while((trim=sline.find("\r"))!=std::string::npos) sline.erase(trim,1);

		size_t begin=0;
		size_t end=0;
		while(true) {
			end=sline.find(",",begin);
			if (end==std::string::npos) end=strlen(line);
			std::string tok = sline.substr(begin,end-begin);
			//
			if (tok.find("$")!=std::string::npos) {
				if (!currentFrame.empty()) {
					if (currentFrame.compare("$GPRMC")==0 || currentFrame.compare("$GNRMC")==0) {
						caps.SetGPRMC();
						unsigned int parsed=0;
						if (!fields[1].empty()) { // Time
							std::stringstream stream(fields[1]);
							float v;
							stream>>v;
							lastGPRMC.SetTime((unsigned int)(v*1000.f));
							++parsed;
						}
						if (!fields[2].empty()) { // Validity
							std::stringstream stream(fields[2]);
							char v;
							stream>>v;
							lastGPRMC.SetValidity(v=='A');
							++parsed;
						}
						if (!(fields[3].empty() || fields[4].empty())) { // Latitude
							std::stringstream stream1(fields[3]);
							std::stringstream stream2(fields[4]);
							float v; stream1>>v;
							char s;  stream2>>s;
							if (s=='S') v=-v;
							convertDegrees(v);
							lastGPRMC.SetLatitude(v);
							++parsed;
						}
						if (!(fields[5].empty() || fields[6].empty())) { // Longitude
							std::stringstream stream1(fields[5]);
							std::stringstream stream2(fields[6]);
							float v; stream1>>v;
							char s;  stream2>>s;
							if (s=='W') v=-v;
							convertDegrees(v);
							lastGPRMC.SetLongitude(v);
							++parsed;
						}
						if (!fields[7].empty()) { // Speed
							std::stringstream stream(fields[7]);
							float v; stream>>v;
							convertKnotsToMetersPerSecond(v);
							lastGPRMC.SetSpeed(v);
							++parsed;
						}
						if (!fields[8].empty()) { // Azimuth
							std::stringstream stream(fields[8]);
							float v; stream>>v;
							lastGPRMC.SetAzimuth(v);
							++parsed;
						}
						if (parsed!=6) lastGPRMC.Reset();
					} else if (currentFrame.compare("$GPGGA")==0 || currentFrame.compare("$GNGGA")==0) {
						caps.SetGPGGA();
						unsigned int parsed=0;
						if (!fields[1].empty()) { // Time
							std::stringstream stream(fields[1]);
							float v; stream>>v;
							lastGPGGA.SetTime((unsigned int)(v*1000.f));
							++parsed;
						}
						if (!(fields[2].empty() || fields[3].empty())) { // Latitude
							++parsed;
						}
						if (!(fields[4].empty() || fields[5].empty())) { // Longitude
							++parsed;
						}
						if (!fields[6].empty()) { // Fix
							++parsed;
						}
						if (!fields[7].empty()) { // Sattelites
							std::stringstream stream(fields[7]);
							unsigned int v; stream>>v;
							lastGPGGA.SetSatellites(v);
							++parsed;
						}
						if (!fields[8].empty()) { // HDOP
							std::stringstream stream(fields[8]);
							float v; stream>>v;
							lastGPGGA.SetHDOP(v);
							++parsed;
						}
						if (parsed!=6) lastGPGGA.Reset();
					}
				}
				currentFrame=tok;
				fields.clear();
			}
			fields.push_back(tok);
			//
			begin=end+1;
			if (begin>=strlen(line)) break;
		}
	
		bool complete=true;
		if (caps.HasGPRMC()) complete = complete && !lastGPRMC.Empty();
		if (caps.HasGPGGA()) complete = complete && !lastGPGGA.Empty();
		if (complete) {
			SPosition newPos;
			newPos.longitude = lastGPRMC.Longitude();
			newPos.latitude  = lastGPRMC.Latitude();
			newPos.timestamp = lastGPRMC.Time();
			newPos.dop       = lastGPGGA.HDOP();
			newPos.azimuth   = lastGPRMC.Azimuth();
			newPos.speed     = lastGPRMC.Speed();
			newPos.validity  = lastGPRMC.Validity();
			if (newPos.validity) {
				positions.push_back(newPos);
			}
			lastGPRMC.Reset();
			lastGPGGA.Reset();
		}
	}


        std::cout << "<?xml version=\"1.0\" encoding=\"ISO-8859-1\"?>" << std::endl
	<< "<kml xmlns=\"http://earth.google.com/kml/2.0\">" << std::endl
	<< "<Document>" << std::endl
	<< "	<description><![CDATA[NMEA]]></description>" << std::endl
	<< "	<Style id=\"ikon\"><IconStyle><Icon><href>http://maps.google.com/mapfiles/kml/shapes/arrow.png</href></Icon><scale>1.0</scale></IconStyle></Style>" << std::endl;

	// Draw GPS Route
	std::cout << "	<Placemark>" << std::endl
	<< "		<name>" << filename << "</name>" << std::endl
	<< "		<Style>" << std::endl
	<< "			<LineStyle>" << std::endl
	<< "				<color>ff" << "ff00ff" << "</color>" << std::endl
	<< "				<width>4</width>" << std::endl
	<< "			</LineStyle>" << std::endl
	<< "		</Style>" << std::endl
	<< "		<LineString>" << std::endl
	<< "			<tessellate>0</tessellate>" << std::endl
	<< "			<altitudeMode>clampedToGround</altitudeMode>" << std::endl
	<< "			<coordinates>" << std::endl;
	std::vector<SPosition>::iterator it = positions.begin();


	while (it!=positions.end()) {
		SPosition pos(*it);
		std::cout << "				" << pos.longitude << "," << pos.latitude << ",0" << std::endl;
		++it;
	}

	std::cout << "			</coordinates>" << std::endl
	<< "		</LineString>" << std::endl
	<< "	</Placemark>" << std::endl;
	//

#if 1
	// Plot info
	it = positions.begin();
	unsigned int index=0;
	while (it!=positions.end()) {
		SPosition pos(*it);
		std::cout << "	<Placemark>" << std::endl
		<< "		<description>" << index
		<< "<br/>xy: " << pos.longitude << "," << pos.latitude
		<< "<br/>Azimuth: " << pos.azimuth
		<< "<br/>Speed: " << pos.speed << "m/s"
		<< "<br/>HDOP: " << pos.dop
		<< "<br/>Time: " << .001f*(float)pos.timestamp
		<< "<br/>Validity: " << (pos.validity?"true":"false")
		<< "</description>" << std::endl
		<< "		<Style><IconStyle><Icon><href>http://maps.google.com/mapfiles/kml/shapes/arrow.png</href></Icon><scale>0.67</scale><heading>" << (180.f+pos.azimuth) << "</heading></IconStyle></Style>" << std::endl
		<< "		<LookAt>" << std::endl
		<< "			<longitude>" << pos.longitude << "</longitude>" << std::endl
		<< "			<latitude>" << pos.latitude << "</latitude>" << std::endl
		<< "			<heading>" << pos.azimuth << "</heading>" << std::endl
		<< "			<range>50.0</range>" << std::endl
		<< "			<tilt>0.0</tilt>" << std::endl
		<< "		</LookAt>" << std::endl
		<< "		<Point>" << std::endl
		<< "			<coordinates>" << pos.longitude << "," << pos.latitude << ",0</coordinates>" << std::endl
		<< "		</Point>" << std::endl
		<< "	</Placemark>" << std::endl;
		++it;
		++index;
	}
#endif

	//
	std::cout << "</Document>" << std::endl
	<< "</kml>" << std::endl;
	ifs.close();
	return 0;
}

