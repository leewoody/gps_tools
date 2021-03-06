// Base classes for NMEA parsingaaa

#define _USE_MATH_DEFINES
#include <iostream>
#include <string>
#include <fstream>
#include <sstream>
#include <vector>
#include <list>
#include <cmath>
#include "math.h"

class CCapabilities {
private:
    unsigned m_GPRMC : 1;
    unsigned m_GPGGA : 1;
public:
    inline CCapabilities() : m_GPRMC(0), m_GPGGA(0) {
    }
    inline void SetGPRMC() {
        m_GPRMC=1;
    }
    inline void SetGPGGA() {
        m_GPGGA=1;
    }
    inline bool HasGPRMC() const {
        return (m_GPRMC!=0);
    }
    inline bool HasGPGGA() const {
        return (m_GPGGA!=0);
    }
};

// GPRMC nmea sentence
class CGPRMC {
private:
    unsigned int m_Time;
    bool m_Validity;
    double m_Latitude;
    double m_Longitude;
    float m_Speed;
    float m_Azimuth;
	std::string m_Mode;
	std::string m_sTime;
public:
    inline CGPRMC();
    // getters
    inline unsigned int     Time() const;
    inline bool             Validity() const;
    inline double            Latitude() const;
    inline double            Longitude() const;
    inline float            Speed() const;
    inline float            Azimuth() const;
	inline std::string      Mode() const;
	inline std::string      sTime() const;
    // setters
    inline void             SetTime(unsigned int);
	inline void             SetSTime(std::string);
    inline void             SetValidity(bool);
    inline void             SetLatitude(float);
    inline void             SetLongitude(float);
    inline void             SetSpeed(float);
    inline void             SetAzimuth(float);
	inline void             SetMode(std::string);
    inline void             Reset();
    // queryers
    inline bool             HasTime() const;
    inline bool             HasValidity() const;
    inline bool             HasLatitude() const;
    inline bool             HasLongitude() const;
    inline bool             HasSpeed() const;
    inline bool             HasAzimuth() const;
	inline bool             HasMode() const;
    inline bool             Empty() const;
private:
    inline CGPRMC(const CGPRMC&) {
    }
    inline CGPRMC& operator=(const CGPRMC&) {
        return *this;
    }
};

// GPGGA nmea sentence
class CGPGGA {
private:
    unsigned int m_Time;
    float m_HDOP;
    int m_Satellites;
public:
    inline CGPGGA();
    // getters
    inline unsigned int     Time() const;
    inline float            HDOP() const;
    inline int              Satellites() const;
    // setters
    inline void             SetTime(unsigned int);
    inline void             SetHDOP(float);
    inline void             SetSatellites(int);
    inline void             Reset();
    // queryers
    inline bool             HasTime() const;
    inline bool             HasHDOP() const;
    inline bool             HasSatellites() const;
    inline bool             Empty() const;
private:
    inline CGPGGA(const CGPGGA&) {
    }
    inline CGPGGA& operator=(const CGPGGA&) {
        return *this;
    }
};

// GPRMC
CGPRMC::CGPRMC() {
    Reset();
}
inline unsigned int CGPRMC::Time() const {
    return m_Time;
}
inline std::string CGPRMC::sTime() const {
	return m_sTime;
}
inline bool CGPRMC::Validity() const {
    return m_Validity;
}
inline double CGPRMC::Latitude() const {
    return m_Latitude;
}
inline double CGPRMC::Longitude() const {
    return m_Longitude;
}
inline float CGPRMC::Speed() const {
    return m_Speed;
}
inline float CGPRMC::Azimuth() const {
    return m_Azimuth;
}
inline std::string CGPRMC::Mode() const {
	return m_Mode;
}
inline void CGPRMC::SetTime(unsigned int v) {
    m_Time=v;
}
inline void CGPRMC::SetSTime(std::string s) {
	m_sTime=s;
}
inline void CGPRMC::SetValidity(bool v)     {
    m_Validity=v;
}
inline void CGPRMC::SetLatitude(float v)    {
    m_Latitude=v;
}
inline void CGPRMC::SetLongitude(float v)   {
    m_Longitude=v;
}
inline void CGPRMC::SetSpeed(float v)       {
    m_Speed=v;
}
inline void CGPRMC::SetAzimuth(float v)     {
    m_Azimuth=v;
}
inline void CGPRMC::SetMode(std::string s)     {
	m_Mode=s;
}
inline void CGPRMC::Reset()                 {
    m_Time          = 0xffffffff;
    m_Validity      = false;
    m_Latitude      = 1000.f;
    m_Longitude     = 1000.f;
    m_Speed         = -1000.f;
    m_Azimuth       = 1000.f;
}
inline bool CGPRMC::HasTime() const {
    return (m_Time!=0xffffffff);
}
inline bool CGPRMC::HasValidity() const {
    return true;
}
inline bool CGPRMC::HasLatitude() const {
    return (m_Latitude!=1000.f);
}
inline bool CGPRMC::HasLongitude() const {
    return (m_Longitude!=1000.f);
}
inline bool CGPRMC::HasSpeed() const {
    return (m_Speed!=-1000.f);
}
inline bool CGPRMC::HasAzimuth() const {
    return (m_Azimuth!=1000.f);
}
inline bool CGPRMC::HasMode() const {
	return (m_Mode!="*");
}
inline bool CGPRMC::Empty() const {
    return (!HasTime()) && (!HasLatitude()) && (!HasLongitude()) && (!HasSpeed()) && (!HasAzimuth());
}

// GPGGA
CGPGGA::CGPGGA()
{
    Reset();
}
inline unsigned int CGPGGA::Time() const {
    return m_Time;
}
inline float CGPGGA::HDOP() const {
    return m_HDOP;
}
inline int CGPGGA::Satellites() const {
    return m_Satellites;
}
inline void CGPGGA::SetTime(unsigned int v)         {
    m_Time=v;
}
inline void CGPGGA::SetHDOP(float v)                {
    m_HDOP=v;
}
inline void CGPGGA::SetSatellites(int v)                {
    m_Satellites=v;
}
inline void CGPGGA::Reset() {
    m_Time          = 0xffffffff;
    m_HDOP          = -1000.f;
    m_Satellites= -1;
}
inline bool CGPGGA::HasTime() const {
    return (m_Time!=0xffffffff);
}
inline bool CGPGGA::HasHDOP() const {
    return (m_HDOP!=-1000.f);
}
inline bool CGPGGA::HasSatellites() const {
    return (m_Satellites!=-1);
}
inline bool CGPGGA::Empty() const {
    return (!HasTime()) && (!HasHDOP()) && (!HasSatellites());
}


// Main program

CCapabilities caps;
CGPRMC lastGPRMC;
CGPRMC lastGPRMCIncludeEmpty;
CGPGGA lastGPGGA;

// Convert NMEA angles to decimal degrees
static void convertDegrees(double& f) {
    int degrees = (int)(f/100.0f);
    float minutes = f - degrees*100.0f;
    minutes = ((minutes*100.0f)/6000.0f);
    f = degrees + minutes;
}

static void convertKnotsToMetersPerSecond(float& v) {
    v*=.5144444444444444f;
}

// (longitude,latitude) <-> meters (spherical projection)

static const float degreeArcLength      = 111226.29991434248924368723f;
static const float degreeArcLengthRec    = .00000899067936962857f;
void convertWGS84ToSI(float fLongitude, float fLatitude, float& fCoordX, float& fCoordY) {
    if (fabsf(fLatitude)>85.f)
        fLatitude = (fLatitude<.0f) ? -85.f : 85.f;
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
    SPosition() : longitude(.0f), latitude(.0f), azimuth(.0f), speed(.0f), dop(.0f), timestamp(""), validity(false), line(0) {
    }
    SPosition(const SPosition& o) : longitude(o.longitude), latitude(o.latitude), azimuth(o.azimuth), speed(o.speed), dop(o.dop), timestamp(o.timestamp), validity(o.validity), line(o.line), mode(o.mode), EDirection(o.EDirection), Case(o.Case) {
    }
    double longitude;
    double latitude;
    float azimuth;
    float speed;
    float dop;
    std::string timestamp;
    bool validity;
    int line;
	std::string mode;
	double EDirection;
	std::string Case;
	double Distance(const SPosition& o){                               //物件的引用，返回兩點距離
		return sqrt(pow(longitude-o.longitude,2)+pow(latitude-o.latitude,2)); 
	}
};

bool positionIsEmpty(const SPosition& o){
	return (o.latitude==1000.f) && (o.longitude==1000.f);
}

double round(double r)
{
	return (r > 0.0) ? floor(r + 0.5) : ceil(r - 0.5);
}
/** 
 * google maps的脚本里代码 
 */    
double EARTH_RADIUS = 6378.137; 
double rad(double d) 
{ 
	return d * M_PI / 180.0; 
}  

/** 
 * 根据两点间经纬度坐标（double值），计算两点间距离，单位为米 
 */ 
double GetDistance(double lat1, double lng1, double lat2, double lng2) 
{ 
    double radLat1 = rad(lat1); 
    double radLat2 = rad(lat2); 
    double a = radLat1 - radLat2; 
    double b = rad(lng1) - rad(lng2); 
	double s = 2 * asin(sqrt(pow(sin(a/2),2) + cos(radLat1)*cos(radLat2)*pow(sin(b/2),2))); 
    s = s * EARTH_RADIUS; 
	s = round(s * 10000) / 10000; 
    return s*1000; 
} 

void ouputGPRMCFields(std::ofstream& ocfs, std::vector<std::string> GPRMCFields){
	std::stringstream stream;
	for (int j=0;j<GPRMCFields.size();j++)
	{
		stream << GPRMCFields[j] << ",";
	}
	ocfs << stream.str() << std::endl;
};

int GetHeadingDiff(int Az1,int Az2)
{
	if (Az1 - Az2 > 180){
		if (Az1>180)
		{
			Az1 = -(360)+Az1;
		}
		if (Az2>180)
		{
			Az2 = -(360)+Az2;
		}
	}
	return std::abs(Az1 - Az2);
}

double GetDirection(const SPosition& o1, const SPosition& o2)
{
	double lat1 = o1.latitude;
	double lng1 = o1.longitude;
	double lat2 = o2.latitude;
	double lng2 = o2.longitude;
	if (lat1==lat2 && lng1==lng2)
	{
		return o1.EDirection;
	}
	if (lng1==lng2)
	{
		if (lat1<lat2)
		{
			return 0;
		}
		else
		{
			return 180;
		}
		
	}
	if (lat1==lat2)
	{
		if (lng1<lng2)
		{
			return 90;
		}
		else
		{
			return 270;
		}
		
	}

	double degree = atan((lat2-lat1)/(lng2-lng1))*180/M_PI;
	if (lng1<lng2)
	{
		return -(degree)+90;
	}
	else
	{
		return 360-(90+degree);
	}
}

#include <iostream>
using namespace std;

// http://annheilong.pixnet.net/blog/post/24919514-%E3%80%90%E9%9B%BB%E8%85%A6%E3%80%91nmea%E6%A8%99%E6%BA%96%E6%A0%BC%E5%BC%8F
int main(int argc, char** argv) {

	//const char* filename = "nmeaEmpty.txt";
	std::string ifilename="";

	//參數個數=3:工程名\讀取txt文件\寫入txt文件
	std::cout<<"total argc=" <<argc << "\n";
	
	for(int i=0; i<argc; i++)
	{
		std::cout<<"argc="<<i<<" \n";
		std::cout<<"argv="<<argv[i]<< "\n";   
		if(i==1)
			ifilename=argv[1];

	}

	//system("PAUSE");

	if(!(ifilename.length() > 0))
	{
		std::cout << "Please enter the NMEA Filename under same folder,\n example:EMER190424-095534.NMEA \n or Absolute Path, example:C:\\EMER190424-095534.NMEA \n";
		getline(std::cin, ifilename);
	}

    //char* filenameExtension = ".txt";
	char oCheckfilename[256];
	char* outCheckfilenameExtension = ".log";
	char outnmeafilterfile[256];
    char* outnmeafilterfileExtension="_filter.kml";
	char outnmeafile[256];
	char outnmea_rmcfile[256];
	char* outfile_rmc = "_rmc.log";

	const char* filename = ifilename.c_str();

	const char* outfileExtension=".kml";

	std::vector<std::string> HandingDoubleQuotes;
	std::vector<std::string> ifilenamefields;
	std::string strfilename;

	if(strspn(ifilename.c_str(), "://") == 0) {
		size_t begin=0;
		size_t end=0;

		while(true) {
			end = ifilename.find("\"", begin);
			std::string tok = ifilename.substr(begin, end-begin);
			HandingDoubleQuotes.push_back(tok);

			begin=end+1;
			if (end==std::string::npos) break;
		}
		if (HandingDoubleQuotes.size()>1)
		{
			ifilename=HandingDoubleQuotes[1];
		}

		begin=0;
		end=0;
		//std::string s = ifilename;
		while(true) {
			end=ifilename.find("\\",begin);
			
			std::string tok = ifilename.substr(begin,end-begin);

			ifilenamefields.push_back(tok);
			//
			begin=end+1;
			if (end==std::string::npos) break;
		}
		strfilename = ifilenamefields[ifilenamefields.size()-1];
	}
	else
	{
		strfilename = ifilename;
	}

	//std::string ifilenameNoExtension = strfilename.substr(0, strcspn(strfilename.c_str(), "."));

	int iCase1 =0; 
	int iCase2 =0;
	int iCase3 =0;
	int iCase4 =0;

	strcpy(oCheckfilename, strfilename.c_str());
	strcpy(outnmeafilterfile, strfilename.c_str());
	strcpy(outnmeafile, strfilename.c_str());
	strcpy(outnmea_rmcfile, strfilename.c_str());

	sprintf(oCheckfilename, "%s_%d_%d_%d_%d",strfilename.c_str(), iCase1,iCase2,iCase3,iCase4);

	strcat(oCheckfilename, outCheckfilenameExtension);
	strcat(outnmeafilterfile, outnmeafilterfileExtension);
	strcat(outnmeafile, outfileExtension);
	strcat(outnmea_rmcfile, outfile_rmc);

	std::ofstream ocfs(oCheckfilename);
	if (!ocfs.is_open()) return 1;

	std::ofstream ofnmeafs(outnmeafilterfile);
	if (!ofnmeafs.is_open()) return 1;

	std::ofstream onmeafs(outnmeafile);
	if (!onmeafs.is_open()) return 1;

	int countline = 0;

    std::cerr << "Converting file \"" << filename << "\"" << std::endl;

    std::vector<SPosition> positions;
	std::vector<SPosition> positionsIncludeEmpty;
	std::vector<SPosition> positionsIncludeEmptyFilter;
    ocfs.precision(6);
    ocfs.precision(20);
	ofnmeafs.precision(6);
	ofnmeafs.precision(20);
	onmeafs.precision(6);
	onmeafs.precision(20);
    std::ifstream ifs(filename);
	std::ofstream ifs_rmc(outnmea_rmcfile);

    if (!ifs.is_open()) return 1;
    char line[512];

    std::vector<std::string> fields;
    std::string currentFrame;

    std::vector<std::vector<std::string>> GPRMCAllFields;

    while (ifs.good()) {
        ifs.getline(line,512, '\n');
        std::string sline(line);
        size_t trim=0;
        while((trim=sline.find("\r"))!=std::string::npos) sline.erase(trim,1);

		//std::cout<< sline <<" \n";

		if (sline.find("RMC")!=std::string::npos) {
			ifs_rmc << sline  ;
			ifs_rmc << "\n";
		}
        size_t begin=0;
        size_t end=0;
        while(true) {
            end=sline.find(",",begin);
            if (end==std::string::npos) end=strlen(line);
            std::string tok = sline.substr(begin,end-begin);
            //
            /*
				「RMC」=>GPS建議最小傳輸資料
				$--RMC,hhmmss.ss,A,llll.ll,a,yyyyy.yy,a,x.x,x.x,xxxxxx,x.x,a*hh
				($GPRMC,<1>,<2>,<3>,<4>,<5>,<6>,<7>,<8>,<9>,<10>,<11>)
				範例說明：
				$GPRMC,055148,A,2407.8945,N,12041.7649,E,000.0,000.0,061196,003.1,W,A*69

				1) $GPRMC,055148 接收定位時間（UTC time）格式：時時分分秒秒.秒秒秒（hhmmss.sss）。 
				2) A = 定位狀態，A：資料可用，V：資料不可用。 
				3) 2407.8945 = 緯度，格式：度度分分.分分分分（ddmm.mmmm）。 
				4) N = 緯度區分，北半球（N）或南半球（S）。 
				5) 12041.7649 = 經度，格式：度度分分.分分分分。 
				6) E = 經度區分，東（E）半球或西（W）半球。 
				7) 000.0 = 相對航行速度， 0.0 至 1851.8 knots(節)
				8) 000.0 = 相對航行方向，000.0 至 359.9度。實際值。 
				9) 061196 = 日期，格式：日日月月年年（ddmmyy）。 
				10) 003.1 = 磁極變量，000.0 至180.0度。 
				11) W = 磁方位角（西W或東E）度數。 
				12) A*hh = Checksum.(檢查位元)
					A = Mode indicator
					‘N’ = Data not valid
					‘A’ = Autonomous mode
					‘D’ = Differential mode
					‘E’ = Estimated (dead reckoning) mode 
             */
            if (tok.find("$")!=std::string::npos) {
                if (!currentFrame.empty()) {
                    if (currentFrame.compare("$GPRMC")==0 || currentFrame.compare("$GNRMC")==0) {
                        caps.SetGPRMC();
						unsigned int parsed=0;
						GPRMCAllFields.push_back(fields);
                        if (!fields[1].empty()) {                         // Time
                            std::stringstream stream(fields[1]);
//                             char tempBuf[4];
// 							memset(tempBuf, '\0', sizeof(tempBuf));
// 							strncpy(tempBuf, field, 2);
// 							rmcMessage.time.hours = (uint8_t)(atoi(tempBuf));
// 
// 							memset(tempBuf, '\0', sizeof(tempBuf));
// 							memcpy(tempBuf, &field[2], 2);
// 							rmcMessage.time.minutes = (uint8_t)atoi(tempBuf);
// 
// 							memset(tempBuf, '\0', sizeof(tempBuf));
// 							memcpy(tempBuf, &field[4], 2);
// 							rmcMessage.time.seconds = (uint8_t)atoi(tempBuf);
// 
// 							memset(tempBuf, '\0', sizeof(tempBuf));
// 							if (strlen(field) > 9)
// 								memcpy(tempBuf, &field[7], 3);
// 							else
// 								memcpy(tempBuf, &field[7], 2);
// 							rmcMessage.time.milliseconds = (uint16_t)atoi(tempBuf);

							//float v;
                            //stream>>v;
							//lastGPRMC.SetTime((unsigned int)(v*1000.f));
							std::string s; stream>>s;
                            lastGPRMC.SetSTime(s);
                            ++parsed;
						}
                        if (!fields[2].empty()) {                         // Validity
                            std::stringstream stream(fields[2]);
                            char v;
                            stream>>v;
                            lastGPRMC.SetValidity(v=='A');
                            ++parsed;
                        }
                        if (!(fields[3].empty() || fields[4].empty())) {                         // Latitude
                            std::stringstream stream1(fields[3]);
                            std::stringstream stream2(fields[4]);
                            double v; stream1>>v;
                            char s;  stream2>>s;
                            if (s=='S') v=-v;
                            convertDegrees(v);
                            lastGPRMC.SetLatitude(v);
                            ++parsed;
                        }
                        if (!(fields[5].empty() || fields[6].empty())) {                         // Longitude
                            std::stringstream stream1(fields[5]);
                            std::stringstream stream2(fields[6]);
                            double v; stream1>>v;
                            char s;  stream2>>s;
                            if (s=='W') v=-v;
                            convertDegrees(v);
                            lastGPRMC.SetLongitude(v);
                            ++parsed;
                        }
                        if (!fields[7].empty()) {                         // Speed
                            std::stringstream stream(fields[7]);
                            float v; stream>>v;
                            convertKnotsToMetersPerSecond(v);
                            lastGPRMC.SetSpeed(v);
                            ++parsed;
                        }
                        if (!fields[8].empty()) {                         // Azimuth
                            std::stringstream stream(fields[8]);
                            float v; stream>>v;
                            lastGPRMC.SetAzimuth(v);
                            ++parsed;
                        }
						if (!fields[9].empty()) {                         // Azimuth
							//std::stringstream stream(fields[9]);
							std::string s = lastGPRMC.sTime().c_str(); 
							s.append("-");
							s.append(fields[9]);
							//stream >>s;
							lastGPRMC.SetSTime(s);
							++parsed;
						}
						if(fields.size()==13) {
							if (!fields[12].empty())
							{
								std::stringstream stream(fields[12]);
								std::string s; stream>>s;
								s = s.substr(0,1);
								lastGPRMC.SetMode(s);
								++parsed;
							}
						}

						SPosition newPos;
						newPos.longitude = lastGPRMC.Longitude();
						newPos.latitude  = lastGPRMC.Latitude();
						newPos.timestamp = lastGPRMC.sTime();
						newPos.azimuth   = lastGPRMC.Azimuth();
						newPos.speed     = lastGPRMC.Speed();
						newPos.validity  = lastGPRMC.Validity();
						if (lastGPRMC.HasMode())
						{
							newPos.mode  = lastGPRMC.Mode();
						}
						newPos.line      = countline;
						positionsIncludeEmpty.push_back(newPos);

                        if (fields.size()==13 && parsed!=7) lastGPRMC.Reset();
						if (fields.size()==12 && parsed!=6) lastGPRMC.Reset();
                    } else if (currentFrame.compare("$GPGGA")==0 || currentFrame.compare("$GNGGA")==0) {
                        caps.SetGPGGA();
                        unsigned int parsed=0;
                        if (!fields[1].empty()) {                         // Time
                            std::stringstream stream(fields[1]);
                            float v; stream>>v;
                            lastGPGGA.SetTime((unsigned int)(v*1000.f));
                            ++parsed;
                        }
                        if (!(fields[2].empty() || fields[3].empty())) {                         // Latitude
                            ++parsed;
                        }
                        if (!(fields[4].empty() || fields[5].empty())) {                         // Longitude
                            ++parsed;
                        }
                        if (!fields[6].empty()) {                         // Fix
                            ++parsed;
                        }
                        if (!fields[7].empty()) {                         // Sattelites
                            std::stringstream stream(fields[7]);
                            unsigned int v; stream>>v;
                            lastGPGGA.SetSatellites(v);
                            ++parsed;
                        }
                        if (!fields[8].empty()) {                         // HDOP
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
            newPos.timestamp = lastGPRMC.sTime();
            newPos.dop       = lastGPGGA.HDOP();
            newPos.azimuth   = lastGPRMC.Azimuth();
            newPos.speed     = lastGPRMC.Speed();
            newPos.validity  = lastGPRMC.Validity();
			if (lastGPRMC.HasMode())
			{
				newPos.mode  = lastGPRMC.Mode();
			}
			newPos.line      = countline;
            if (newPos.validity) {
                positions.push_back(newPos);
				if (positions.size()>1)
				{
					positions[positions.size()-1].EDirection = GetDirection(positions[positions.size()-2], positions[positions.size()-1]);
					if (positions[0].EDirection == 0.f)
					{
						positions[0].EDirection = positions[1].EDirection;
					}
				}
            }
            lastGPRMC.Reset();
            lastGPGGA.Reset();
        }
		countline = countline + 1;
    }

	std::cout << "output file. \n";

	for (int i=0; i<positionsIncludeEmpty.size();i++)
	{
		if (i>0 && 
			!positionIsEmpty(positionsIncludeEmpty[i-1]) && 
			!positionIsEmpty(positionsIncludeEmpty[i]))
		{
			if (positionsIncludeEmpty[0].EDirection == 0.f)
			{
				positionsIncludeEmpty[0].EDirection = positionsIncludeEmpty[1].EDirection;
			}
			positionsIncludeEmpty[i].EDirection = GetDirection(positionsIncludeEmpty[i-1],positionsIncludeEmpty[i]);
		}

		//case1
		if (!positionsIncludeEmpty[i].validity && !positionIsEmpty(positionsIncludeEmpty[i]))
		{
			//ocfs << std::endl;
			iCase1++;
			ocfs << "case1,line." << positionsIncludeEmpty[i].line << "   : ";
			ouputGPRMCFields(ocfs, GPRMCAllFields[i]);
			positionsIncludeEmptyFilter.push_back(positionsIncludeEmpty[i]);
			positionsIncludeEmptyFilter[positionsIncludeEmptyFilter.size()-1].Case = "1, Validity=V, But the information is not empty";
		}
		//case2
		if (i>0 && positionsIncludeEmpty[i-1].validity && !positionsIncludeEmpty[i].validity && positionIsEmpty(positionsIncludeEmpty[i]))
		{
			//ocfs << std::endl;
			bool findCase2 = false;

			iCase2++;
			ocfs << "case2a,line." << positionsIncludeEmpty[i-1].line << "   : ";
			ouputGPRMCFields(ocfs, GPRMCAllFields[i-1]);

			for (int j=1;j<=10;j++)
			{
				if (positionsIncludeEmpty[i+j].validity)
				{
					findCase2 = true;

					iCase2++;
					ocfs << "case2b,line." << positionsIncludeEmpty[i+j].line << "   : ";
					ouputGPRMCFields(ocfs, GPRMCAllFields[i+j]);
					positionsIncludeEmptyFilter.push_back(positionsIncludeEmpty[i-1]);
					positionsIncludeEmptyFilter[positionsIncludeEmptyFilter.size()-1].Case = "2a, Validity: A -> V";
                    positionsIncludeEmptyFilter.push_back(positionsIncludeEmpty[i+j]);
					positionsIncludeEmptyFilter[positionsIncludeEmptyFilter.size()-1].Case = "2b, Validity: V -> A";
					break;
				}
			}
			if (!findCase2)
			{
				ocfs << "After the validity value changes from A to V, the next 10 data are not A." << std::endl;
			}
		}
		//case3
		if (i>0 && 
			!positionIsEmpty(positionsIncludeEmpty[i-1]) && 
			!positionIsEmpty(positionsIncludeEmpty[i]))
		{
			if (GetHeadingDiff(positionsIncludeEmpty[i].azimuth, positionsIncludeEmpty[i].EDirection) >= 90)
			{
				//ocfs << std::endl;

				iCase3++;
				ocfs << "case3,line." << positionsIncludeEmpty[i].line << "   : ";
				ouputGPRMCFields(ocfs, GPRMCAllFields[i]);
				positionsIncludeEmptyFilter.push_back(positionsIncludeEmpty[i]);
				positionsIncludeEmptyFilter[positionsIncludeEmptyFilter.size()-1].Case = "3, COG Reverse";
			}
		}
		//case4
		//if (i>0) ocfs << positionsIncludeEmpty[i-1].line << "," << positionsIncludeEmpty[i].line << "," << "Distance = " << GetDistance(positionsIncludeEmpty[i].latitude,positionsIncludeEmpty[i].longitude,positionsIncludeEmpty[i-1].latitude,positionsIncludeEmpty[i-1].longitude) << std::endl;
		if (i>0 && 
			!positionIsEmpty(positionsIncludeEmpty[i-1]) && 
			!positionIsEmpty(positionsIncludeEmpty[i]) && 
			GetDistance(positionsIncludeEmpty[i].latitude, positionsIncludeEmpty[i].longitude, positionsIncludeEmpty[i-1].latitude, positionsIncludeEmpty[i-1].longitude) >= 100)
		{
			//ocfs << std::endl;

			iCase4++;
			ocfs << "case4a,line." << positionsIncludeEmpty[i-1].line << "   : ";
			ouputGPRMCFields(ocfs, GPRMCAllFields[i-1]);
			positionsIncludeEmptyFilter.push_back(positionsIncludeEmpty[i-1]);
			positionsIncludeEmptyFilter[positionsIncludeEmptyFilter.size()-1].Case = "4a, Start jumping point";

			ocfs << "case4b,line." << positionsIncludeEmpty[i].line << "   : ";
			ouputGPRMCFields(ocfs, GPRMCAllFields[i]);
			positionsIncludeEmptyFilter.push_back(positionsIncludeEmpty[i]);
			positionsIncludeEmptyFilter[positionsIncludeEmptyFilter.size()-1].Case = "4b, End jumping point";
		}
	}

	ocfs.close();
	char oCheckOutputReName[256];
	sprintf(oCheckOutputReName, "%s_Case1-%d_Case2-%d_Case3-%d_Case4-%d",strfilename.c_str(), iCase1,iCase2,iCase3,iCase4);
	strcat(oCheckOutputReName, outCheckfilenameExtension);

	std::remove(oCheckOutputReName);

	if (std::rename(oCheckfilename, oCheckOutputReName)) {
		std::perror("Error renaming");
		//return 1;
	}

	onmeafs << "<?xml version=\"1.0\" encoding=\"ISO-8859-1\"?>" << std::endl
		<< "<kml xmlns=\"http://earth.google.com/kml/2.0\">" << std::endl
		<< "<Document>" << std::endl
		<< "	<description><![CDATA[NMEA]]></description>"<< std::endl
		<< "	<Style id=\"ikon\"><IconStyle><Icon><href>http://maps.google.com/mapfiles/kml/shapes/arrow.png</href></Icon><scale>1.0</scale></IconStyle></Style>"<< std::endl;
 
	ofnmeafs << "<?xml version=\"1.0\" encoding=\"ISO-8859-1\"?>" << std::endl
		<< "<kml xmlns=\"http://earth.google.com/kml/2.0\">" << std::endl
		<< "<Document>" << std::endl
		<< "	<description><![CDATA[NMEA]]></description>"<< std::endl
		<< "	<Style id=\"ikon\"><IconStyle><Icon><href>http://maps.google.com/mapfiles/kml/shapes/arrow.png</href></Icon><scale>1.0</scale></IconStyle></Style>"<< std::endl;

	// Draw GPS Route
	onmeafs << "	<Placemark>"<< std::endl
		<< "		<name>"<< filename << "</name>" << std::endl
		<< "		<Style>"<< std::endl
		<< "			<LineStyle>"<< std::endl
		<< "				<color>ff"<< "ff00ff" << "</color>" << std::endl
		<< "				<width>4</width>"<< std::endl
		<< "			</LineStyle>"<< std::endl
		<< "		</Style>"<< std::endl
		<< "		<LineString>"<< std::endl
		<< "			<tessellate>0</tessellate>"<< std::endl
		<< "			<altitudeMode>clampedToGround</altitudeMode>"<< std::endl
		<< "			<coordinates>"<< std::endl;

	ofnmeafs << "	<Placemark>"<< std::endl
		<< "		<name>"<< filename << "</name>" << std::endl
		<< "		<Style>"<< std::endl
		<< "			<LineStyle>"<< std::endl
		<< "				<color>ff"<< "ff00ff" << "</color>" << std::endl
		<< "				<width>4</width>"<< std::endl
		<< "			</LineStyle>"<< std::endl
		<< "		</Style>"<< std::endl
		<< "		<LineString>"<< std::endl
		<< "			<tessellate>0</tessellate>"<< std::endl
		<< "			<altitudeMode>clampedToGround</altitudeMode>"<< std::endl
		<< "			<coordinates>"<< std::endl;

	//std::vector<SPosition>::iterator it1 = positions.begin();
	std::vector<SPosition>::iterator it1 = positionsIncludeEmpty.begin();

	//while (it1!=positions.end()) {
	while (it1!=positionsIncludeEmpty.end()) {
		SPosition pos(*it1);
		onmeafs << "				"<< pos.longitude << "," << pos.latitude << ",0" << std::endl;
		++it1;
	}

	std::vector<SPosition>::iterator it2 = positionsIncludeEmptyFilter.begin();

	while (it2!=positionsIncludeEmptyFilter.end()) {
		SPosition pos(*it2);
		ofnmeafs << "				"<< pos.longitude << "," << pos.latitude << ",0" << std::endl;
		++it2;
	}


	onmeafs << "			</coordinates>"<< std::endl
		<< "		</LineString>"<< std::endl
		<< "	</Placemark>"<< std::endl; 

	ofnmeafs << "			</coordinates>"<< std::endl
        << "		</LineString>"<< std::endl
        << "	</Placemark>"<< std::endl;
    //

#if 1
    // Plot info
    //it1 = positions.begin();
	it1 = positionsIncludeEmpty.begin();
	it2 = positionsIncludeEmptyFilter.begin();
	unsigned int index=0;
	std::string CaseImg ="<Icon><href>http://maps.google.com/mapfiles/kml/shapes/arrow.png";

	while (it1!=positionsIncludeEmpty.end()) {
		SPosition pos(*it1);
		
		if (pos.Case.find("1")!=std::string::npos) {
			CaseImg = "<Icon><href>http://maps.google.com/mapfiles/kml/paddle/1.png";
		}else if(pos.Case.find("2")!=std::string::npos) {
			CaseImg = "<Icon><href>http://maps.google.com/mapfiles/kml/paddle/2.png";
		}else if(pos.Case.find("3")!=std::string::npos) {
			CaseImg = "<Icon><href>http://maps.google.com/mapfiles/kml/paddle/3.png";
		}else if(pos.Case.find("4")!=std::string::npos) {
			CaseImg = "<Icon><href>http://maps.google.com/mapfiles/kml/paddle/4.png";
		}else{
			CaseImg ="<Icon><href>http://maps.google.com/mapfiles/kml/shapes/arrow.png";
		}

		if(pos.mode == "E")
		{
			CaseImg = "<color>ff0000ff</color>" + CaseImg ;
		}

		onmeafs << "	<Placemark>"<< std::endl
			<< "		<description>"<< index
			<< "<br/>xy: " << pos.longitude << "," << pos.latitude
			<< "<br/>Azimuth: " << pos.azimuth
			<< "<br/>Estimated Direction: " << pos.EDirection
			<< "<br/>Speed: " << pos.speed << "m/s"
			<< "<br/>HDOP: " << pos.dop
			<< "<br/>Time: " << pos.timestamp
			<< "<br/>Validity: " << (pos.validity ? "true" : "false")
			<< "<br/>DR Mode: " << (pos.mode == "E" ? "true" : "false")
			<< "</description>" << std::endl
			<< "		<Style><IconStyle>" << CaseImg << "</href></Icon><scale>" << (pos.mode == "E" ? "0.8" : "0.67") << "</scale><heading>" <<  180.f+pos.azimuth << "</heading></IconStyle></Style>" << std::endl
			<< "		<LookAt>"<< std::endl
			<< "			<longitude>"<< pos.longitude << "</longitude>" << std::endl
			<< "			<latitude>"<< pos.latitude << "</latitude>" << std::endl
			<< "			<heading>"<< pos.azimuth << "</heading>" << std::endl
			<< "			<range>50.0</range>"<< std::endl
			<< "			<tilt>0.0</tilt>"<< std::endl
			<< "		</LookAt>"<< std::endl
			<< "		<Point>"<< std::endl
			<< "			<coordinates>"<< pos.longitude << "," << pos.latitude << ",0</coordinates>" << std::endl
			<< "		</Point>"<< std::endl
			<< "	</Placemark>"<< std::endl;
		++it1;
		++index;
	}

	index=0;
	while (it2!=positionsIncludeEmptyFilter.end()) {
        SPosition pos(*it2);

		if (pos.Case.find("1")!=std::string::npos) {
			CaseImg = "<Icon><href>http://maps.google.com/mapfiles/kml/paddle/1.png";
		}else if(pos.Case.find("2")!=std::string::npos) {
			CaseImg = "<Icon><href>http://maps.google.com/mapfiles/kml/paddle/2.png";
		}else if(pos.Case.find("3")!=std::string::npos) {
			CaseImg = "<Icon><href>http://maps.google.com/mapfiles/kml/paddle/3.png";
		}else if(pos.Case.find("4")!=std::string::npos) {
			CaseImg = "<Icon><href>http://maps.google.com/mapfiles/kml/paddle/4.png";
		}else{
			CaseImg ="<Icon><href>http://maps.google.com/mapfiles/kml/shapes/arrow.png";
		}

		if(pos.mode == "E")
		{
			CaseImg = "<color>ffbf80ff</color>" + CaseImg ;
		}

        ofnmeafs << "	<Placemark>"<< std::endl
                  << "		<description>"<< index
				  << "<br/>Case: " << pos.Case
				  << "<br/>"<< strfilename << " (line. " << pos.line << ")"
                  << "<br/>xy: " << pos.longitude << "," << pos.latitude
                  << "<br/>Azimuth: " << pos.azimuth
				  << "<br/>Estimated Direction: " << pos.EDirection
                  << "<br/>Speed: " << pos.speed << "m/s"
                  //<< "<br/>HDOP: " << pos.dop
                  << "<br/>Time: " << pos.timestamp
                  << "<br/>Validity: " << (pos.validity ? "true" : "false")
				  << "<br/>DR Mode: " << (pos.mode == "E" ? "true" : "false")
                  << "</description>" << std::endl
				  << "		<Style><IconStyle>" << CaseImg << "</href></Icon><scale>" << (pos.mode == "E" ? "0.8" : "0.67") << "</scale><heading>" << 180.f+pos.azimuth << "</heading></IconStyle></Style>" << std::endl
				  << "		<LookAt>"<< std::endl
                  << "			<longitude>"<< pos.longitude << "</longitude>" << std::endl
                  << "			<latitude>"<< pos.latitude << "</latitude>" << std::endl
                  << "			<heading>"<< pos.azimuth << "</heading>" << std::endl
                  << "			<range>50.0</range>"<< std::endl
                  << "			<tilt>0.0</tilt>"<< std::endl
                  << "		</LookAt>"<< std::endl
                  << "		<Point>"<< std::endl
                  << "			<coordinates>"<< pos.longitude << "," << pos.latitude << ",0</coordinates>" << std::endl
                  << "		</Point>"<< std::endl
                  << "	</Placemark>"<< std::endl;
        ++it2;
        ++index;
    }
#endif

	onmeafs << "</Document>" << std::endl
		<< "</kml>" << std::endl;

    ofnmeafs << "</Document>" << std::endl
              << "</kml>" << std::endl;

    ifs.close();
	ifs_rmc.close();
	ofnmeafs.close();
	onmeafs.close();
    return 0;
}

