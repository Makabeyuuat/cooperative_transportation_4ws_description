#include "csvLogger.hpp"
#include "initial.hpp"   // x_old, sr などユーザ側の変数宣言がある想定

#include <cerrno>
#include <cstring>

std::atomic<bool> CSVLogger::sig_close_requested_{false};

CSVLogger::CSVLogger(const std::string& dir, int close_threshold, int sync_every_n)
  : closed_(false),
    close_threshold_(close_threshold),
    sync_every_n_(sync_every_n <= 0 ? 1 : sync_every_n)  // 0/負は毎行同期扱い
{
  // 出力ディレクトリ作成
  try {
    std::filesystem::create_directories(dir);
  } catch (const std::exception& e) {
    ROS_ERROR("CSVLogger: failed to create dir '%s': %s", dir.c_str(), e.what());
  }

  // ファイル名
  const std::string ts = makeTimeStamp();
  const std::string filename = dir + "/gazebo_log_" + ts + ".csv";

  // 高レベルストリーム（テキスト）
  csv_.open(filename, std::ios::out | std::ios::trunc);
  if (!csv_.is_open()) {
    ROS_ERROR("CSVLogger: failed to open '%s'", filename.c_str());
    return;
  }

  // 低レベルFD（同期用）。同じパスを開けば同一inodeなので同期が効く。
  fd_ = ::open(filename.c_str(), O_WRONLY | O_CLOEXEC);
  if (fd_ < 0) {
    ROS_ERROR("CSVLogger: open fd failed for '%s': %s", filename.c_str(), std::strerror(errno));
  }

  // ヘッダー（先頭は t）
  csv_ << "t,x,y,theta0,phi1,theta1,phi2,theta2,phi3,theta3,phi4,theta4,phi5,theta5,phi6,theta6,phi7,theta7,phi8,theta8,phi9,theta9,phi10,theta10,"
       << "x1,Y1,x2,y2,x3,y3,Trix[1],Triy[1],Trix[2],Triy[2],Trix[3],Triy[3],varphi1,varphi2,varphi3,z21,z22,z31,z32,z41,z42,z51,z52,z61,z62,z71,z72,z81,z82,z91,z92,"
       <<"z101,z102,z111,z112,z121,z122,"
       << "al21,al22,al31,al32,al33,al41,al42,al43,al44,al51,al52,al53,al54,al55,al61,al62,al63,al66,al71,al72,al73,al76,al77,al81,al82,al83,al88,al91,al92,al93,al99,al101,al102,al103,al1010,al111,al112,aL113,al1110,al1111,al121,al122,al123,al1210,,al1211,al1212"
       << "u1,u2,u3,u4,u5,u6,u7,u8,u9,u10,u11,u12,v1,v2,v3,PSx,PSy,d,d0d,Q,nh1,nh2,nh3,nh4,nh5,nh6,nh7,nh8,nh9,nh10,"
       << "thetap1d,thetap2d,dthetap2d,thetap3d,dthetap3d,thetap4d,thetap5d,dthetap5d,thetap6d,dthetap6d,thetap7d,thetap8d,dthetap8d,thetap9d,dthetap9d,thetap10d,"
       << "deltap1d,delta2d,delta3d,thetap0,thetap1,thetap2,thetap3,thetap4,thetap5,thetap6,thetap7,thetap8,thetap9,thetap10,thetaT,"
       << "K81,K82,K83,K84,K91,K92,K93,K94,"
       << "sr.Cs, sr.Cs1, 1 - sr.d*sr.Cs,"
       <<"b4,b5,b6,b7,b8,b9,b10,b11,b12"
       << "\n";

  // ヘッダーも確実に同期
  csv_.flush();
  if (fd_ >= 0) ::fdatasync(fd_);

  // SIGINT/SIGTERM のハンドラ設定（安全なやり方：フラグだけ立てる）
  installSignalHandlers();
}

CSVLogger::~CSVLogger() {
  flushAndClose();
}

void CSVLogger::flushAndClose() {
  if (closed_) return;
  try {
    if (csv_.is_open()) {
      csv_.flush();            // ユーザ空間→OSキャッシュ
    }
    if (fd_ >= 0) {
      ::fdatasync(fd_);        // OSキャッシュ→ディスク
      ::close(fd_);
      fd_ = -1;
    }
    if (csv_.is_open()) {
      csv_.close();
    }
    ROS_INFO("CSVLogger: file closed safely.");
  } catch (...) {
    // 例外は握りつぶす（終了時に投げない）
  }
  closed_ = true;
}

void CSVLogger::installSignalHandlers() {
  struct sigaction sa;
  sa.sa_handler = &CSVLogger::handleSignal;
  sigemptyset(&sa.sa_mask);
  sa.sa_flags = 0;

  sigaction(SIGINT,  &sa, nullptr);
  sigaction(SIGTERM, &sa, nullptr);
  // ※ SIGKILL, SIGSTOP は捕捉不可。kill -9 は防げません。
}

void CSVLogger::handleSignal(int /*signum*/) {
  // async-signal-safe な処理のみ：フラグを立てて、ROS終了を促す
  sig_close_requested_.store(true, std::memory_order_relaxed);
  ros::shutdown();
}

void CSVLogger::logData() {
  if (closed_ || !csv_.is_open()) return;

  try {
    const double tnow = ros::Time::now().toSec();

    // ---- 1行書き出し（必要に応じてここにあなたの列を並べる）----
    csv_ << x_old[0] << ',' << x_old[1] << ',' << x_old[2] << ',' << x_old[3] << ',' << x_old[4] << ','<< x_old[5] << ','<< x_old[6] << ',' << x_old[7] << ',' << x_old[8] << ',' << x_old[9] << ',' << x_old[10] << ','<< x_old[11] << ','
         << x_old[12] << ',' << x_old[13] << ',' << x_old[14] << ',' << x_old[15] << ',' << x_old[16] << ','<< x_old[17] << ',' << x_old[18] << ',' << x_old[19] << ',' << x_old[20] << ',' << x_old[21] << ',' << x_old[22] << ','<< x_old[23]
         << ","  << x1
         << ","  << Y1
         << ","  << x2
         << ","  << y2
         << ","  << x3
         << ","  << y3
         << ","  << Trix[1]
         << ","  << Triy[1]
         << ","  << Trix[2]
         << ","  << Triy[2]
         << ","  << Trix[3]
         << ","  << Triy[3]
         << ","  << Phi[1]
         << ","  << Phi[2]
         << ","  << Phi[3]
         << ","  << z21
         << ","  << z22
         << ","  << z31
         << ","  << z32
         << ","  << z41
         << ","  << z42
         << ","  << z51
         << ","  << z52
         << ","  << z61
         << ","  << z62
         << ","  << z71
         << ","  << z72
         << ","  << z81
         << ","  << z82
         << ","  << z91
         << ","  << z92
         << ","  << z101
         << ","  << z102
         << ","  << z111
         << ","  << z112
         << ","  << z121
         << ","  << z122
         << ","  << alpha21
         << ","  << alpha22
         << ","  << alpha31
         << ","  << alpha32
         << ","  << alpha33
         << ","  << alpha41
         << ","  << alpha42
         << ","  << alpha43
         << ","  << alpha44
         << ","  << alpha51
         << ","  << alpha52
         << ","  << alpha53
         << ","  << alpha54
         << ","  << alpha55
         << ","  << alpha61
         << ","  << alpha62
         << ","  << alpha63
         << ","  << alpha66
         << ","  << alpha71
         << ","  << alpha72
         << ","  << alpha73
         << ","  << alpha76
         << ","  << alpha77
         << ","  << alpha81
         << ","  << alpha82
         << ","  << alpha83
         << ","  << alpha88
         << ","  << alpha91
         << ","  << alpha92
         << ","  << alpha93
         << ","  << alpha99
          << ","  << alpha101
          << ","  << alpha102
          << ","  << alpha103
          << ","  << alpha1010
          << ","  << alpha111
          << ","  << alpha112
          << ","  << alpha113
          << ","  << alpha1110
          << ","  << alpha1111
          << ","  << alpha121
          << ","  << alpha122
          << ","  << alpha123
          << ","  << alpha1210
          << ","  << alpha1211
          << ","  << alpha1212
         << ","  << u1
         << ","  << u2
         << ","  << u3
         << ","  << u4
         << ","  << u5
         << ","  << u6
         << ","  << u7
         << ","  << u8
         << ","  << u9
         << ","  << u10
         << ","  << u11
         << ","  << u12
         << ","  << v1
         << ","  << v2
         << ","  << v3
         << ","  << sr.Psx
         << ","  << sr.Psy
         << ","  << sr.d
         << ","  << d0d
         << ","  << sr.j
         << ","  << nh1
         << ","  << nh2
         << ","  << nh3
         << ","  << nh4
         << ","  << nh5
         << ","  << nh6
         << ","  << nh7
         << ","  << nh8
         << ","  << nh9
         << ","  << nh10
         << ","  << thetap1d
         << ","  << thetap2d
         << ","  << dthetap2d
         << ","  << thetap3d
         << ","  << dthetap3d
         << ","  << thetap4d
         << ","  << thetap5d
         << ","  << dthetap5d
         << ","  << thetap6d
         << ","  << dthetap6d
         << ","  << thetap7d
         << ","  << thetap8d
         << ","  << dthetap8d
         << ","  << thetap9d
         << ","  << dthetap9d
         << ","  << thetap10d
         << ","  << delta1d
         << ","  << delta2d
         << ","  << delta3d
         << ","  << Thetap0
         << ","  << Thetap1
         << ","  << Thetap2
         << ","  << Thetap3
         << ","  << Thetap4
         << ","  << Thetap5
         << ","  << Thetap6
         << ","  << Thetap7
         << ","  << Thetap8
         << ","  << Thetap9
         << ","  << Thetap10
         << ","  << thetaT
         << ","  << K81 << "," << K82 << "," << K83 << "," << K84
         << ","  << K91 << "," << K92 << "," << K93 << "," << K94
         << ","  << sr.Cs << "," << sr.Cs1
         << ","  << 1 - sr.d*sr.Cs
         << ","  << b4 << "," << b5 << "," << b6 << "," << b7 << "," << b8 << "," << b9 << "," << b10 << "," << b11 << "," << b12
         << "\n";
    // ---- ここまで1行 ----

    // ユーザ→OSキャッシュへ
    csv_.flush();

    // OSキャッシュ→ディスクへ（間引き可能）
    ++line_counter_;
    if (fd_ >= 0) {
      if (line_counter_ % sync_every_n_ == 0) {
        ::fdatasync(fd_);
      }
    }

    // しきい値でクローズ
    if (close_threshold_ >= 0 && sr.j == close_threshold_) {
      flushAndClose();
      ROS_INFO("CSVLogger: closed CSV file at sr.j = %d", sr.j);
    }

    // シグナルで終了要求が来ていたらクローズ
    if (sig_close_requested_.load(std::memory_order_relaxed)) {
      flushAndClose();
    }

  } catch (const std::exception& e) {
    ROS_ERROR("CSVLogger: write failed: %s", e.what());
  } catch (...) {
    ROS_ERROR("CSVLogger: write failed: unknown error");
  }
}

std::string CSVLogger::makeTimeStamp() {
  std::time_t t = std::time(nullptr);
  std::tm tm = *std::localtime(&t);
  std::ostringstream oss;
  oss << std::put_time(&tm, "%Y%m%d_%H%M%S");
  return oss.str();
}
