#ifndef CSV_LOGGER_HPP
#define CSV_LOGGER_HPP

#include <fstream>
#include <string>
#include <ros/ros.h>
#include <vector>
#include <ctime>
#include <sstream>
#include <iomanip>

class CSVLogger {
public:
  /// @param filename: 出力先ファイル名
  /// @param close_threshold: この j の値になったら自動 close
  CSVLogger(const std::string& filename, int close_threshold = 100000);
  ~CSVLogger();

  /// データ書き出し
  void logData();

private:
  std::ofstream csv_;
  bool         closed_;
  int          close_threshold_;

  /// 起動時刻から "YYYYMMDD_HHMMSS" のタイムスタンプを返す
  static std::string makeTimeStamp();
};

#endif // CSV_LOGGER_H
