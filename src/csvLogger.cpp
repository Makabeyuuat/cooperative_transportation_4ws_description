#include "csvLogger.hpp"
#include "initial.hpp"


CSVLogger::CSVLogger(const std::string& dir, int close_threshold)
  : closed_(false)
  , close_threshold_(close_threshold){
    // タイムスタンプを生成してファイル名を作成
    std::string ts = makeTimeStamp();
    std::string filename = dir + "/data_log_" + ts + ".csv";

    csv_.open(filename, std::ios::out);
    if (!csv_.is_open()) {
      ROS_ERROR("CSVLogger: failed to open '%s'", filename.c_str());
      return;
  }
  // ヘッダー行
  csv_ << "t,x,y,theta0,phi1,theta1,phi2,theta2,phi3,theta3,phi4,theta4,phi5,theta5,phi6,theta6,phi7,theta7,phi8,theta8,phi9,theta9,phi10,theta10,"
       << "x1,Y1,x2,y2,x3,y3,Trix[1],Triy[1],Trix[2],Triy[2],Trix[3],Triy[3],varphi1,varphi2,varphi3,z21,z22,z31,z32,z41,z42,z51,z52,z61,z62,z71,z72,z81,z82,z91,z92,"
       << "al21,al22,al31,al32,al33,al41,al42,al43,al44,al51,al52,al53,al54,al55,al61,al62,al63,al66,al71,al72,al73,al76,al77,al81,al82,al83,al88,al91,al92,al93,al99,"
       << "u1,u2,u3,u4,u5,u6,u7,u8,u9,u10,u11,u12,v1,v2,v3,PSx,PSy,d,d0d,Q,nh1,nh2,nh3,nh4,nh5,nh6,nh7,nh8,nh9,nh10,"
       << "thetap1d,thetap2d,dthetap2d,thetap3d,dthetap3d,thetap4d,thetap5d,dthetap5d,thetap6d,dthetap6d,thetap7d,thetap8d,dthetap8d,thetap9d,dthetap9d,thetap10d,"
       << "delta1d,delta2d,delta3d,thetap0,thetap1,thetap2,thetap3,thetap4,thetap5,thetap6,thetap7,thetap8,thetap9,thetap10,thetaT"
       << "\n";
}

CSVLogger::~CSVLogger() {
  if (csv_.is_open()) csv_.close();
}

void CSVLogger::logData() {
  if (!csv_.is_open()) return;

  double t = ros::Time::now().toSec();
  csv_ << x_old[0] << ',' << x_old[1] << ',' << x_old[2] << ',' << x_old[3] << ',' << x_old[4] << ','<< x_old[5] << ','<< x_old[6] << ',' << x_old[7] << ',' << x_old[8] << ',' << x_old[9] << ',' << x_old[10] << ','<< x_old[11] << ','
  << x_old[12] << ',' << x_old[13] << ',' << x_old[14] << ',' << x_old[15] << ',' << x_old[16] << ','<< x_old[17] << ',' << x_old[18] << ',' << x_old[19] << ',' << x_old[20] << ',' << x_old[21] << ',' << x_old[22] << ','<< x_old[23]
	<<","  << Y1
	<<","  << x2
	<<","  << y2
	<<","  << x3
	<<","  << y3
	<<","  << Trix[1]
	<<","  << Triy[1]
	<<","  << Trix[2]
	<<","  << Triy[2]
	<<","  << Trix[3]
	<<","  << Triy[3]
	<<","  << Phi[1]
	<<","  << Phi[2]
	<<","  << Phi[3]
	<<","  << z21
	<<","  << z22
	<<","  << z31
	<<","  << z32
	<<","  << z41
	<<","  << z42
	<<","  << z51
	<<","  << z52
	<<","  << z61
	<<","  << z62
	<<","  << z71
	<<","  << z72
	<<","  << z81
	<<","  << z82
	<<","  << z91
	<<","  << z92
	<<","  << alpha21
	<<","  << alpha22
	<<","  << alpha31
	<<","  << alpha32
	<<","  << alpha33
	<<","  << alpha41
	<<","  << alpha42
	<<","  << alpha43
	<<","  << alpha44
	<<","  << alpha51
	<<","  << alpha52
	<<","  << alpha53
	<<","  << alpha54
	<<","  << alpha55
	<<","  << alpha61
	<<","  << alpha62
	<<","  << alpha63
	<<","  << alpha66
	<<","  << alpha71
	<<","  << alpha72
	<<","  << alpha73
	<<","  << alpha76
	<<","  << alpha77
	<<","  << alpha81
	<<","  << alpha82
	<<","  << alpha83
	<<","  << alpha88
	<<","  << alpha91
	<<","  << alpha92
	<<","  << alpha93
	<<","  << alpha99
	<<","  << u1
	<<","  << u2
	<<","  << u3
	<<","  << u4
	<<","  << u5
	<<","  << u6
	<<","  << u7
	<<","  << u8
	<<","  << u9
	<<","  << u10
	<<","  << u11
	<<","  << u12
	<<","  << v1
	<<","  << v2
	<<","  << v3
	<<","  << sr.Psx
	<<","  << sr.Psy
	<<","  << sr.d
	<<","  << d0d
	<<"," <<sr.j
	<<"," << nh1
	<<"," << nh2
	<<"," << nh3
	<<"," << nh4
	<<"," << nh5
	<<"," << nh6
	<<"," << nh7
	<<"," << nh8
	<<"," << nh9
	<<"," <<nh10
	<<"," <<theta1d
	<<"," <<thetap2d
	<<"," <<dthetap2d
	<<"," <<thetap3d
	<<"," <<dthetap3d
	<<"," <<theta4d
	<<"," <<thetap5d
	<<"," <<dthetap5d
	<<"," <<thetap6d
	<<"," <<dthetap6d
	<<"," <<theta7d
	<<"," <<thetap8d
	<<"," <<dthetap8d
	<<"," <<thetap9d
	<<"," <<dthetap9d
	<<"," <<theta10d
	<<"," <<delta1d
	<<"," <<delta2d
	<<"," <<delta3d
	<<"," <<Thetap[0]
	<<"," <<Thetap[1]
	<<"," <<Thetap[2]
	<<"," <<Thetap[3]
	<<"," <<Thetap[4]
	<<"," <<Thetap[5]
	<<"," <<Thetap[6]
	<<"," <<Thetap[7]
	<<"," <<Thetap[8]
	<<"," <<Thetap[9]
	<<"," <<Thetap[10]
	<<"," <<thetaT
  << "," << "\n";
  csv_.flush();
  // 閾値に到達したらクローズ
  if (sr.j == close_threshold_) {
    csv_.close();
    closed_ = true;
    ROS_INFO("CSVLogger: closed CSV file at sr.j = %d", sr.j);
  }
}

std::string CSVLogger::makeTimeStamp() {
  std::time_t t = std::time(nullptr);
  std::tm    tm = *std::localtime(&t);
  std::ostringstream oss;
  oss << std::put_time(&tm, "%Y%m%d_%H%M%S");
  return oss.str();
}
