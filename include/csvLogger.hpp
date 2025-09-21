#ifndef CSV_LOGGER_HPP
#define CSV_LOGGER_HPP

#include <atomic>
#include <filesystem>
#include <fstream>
#include <iomanip>
#include <ios>
#include <sstream>
#include <string>
#include <vector>
#include <ctime>

#include <ros/ros.h>

// 低レベル同期用
#include <fcntl.h>   // open
#include <unistd.h>  // fdatasync, close
#include <signal.h>  // sigaction

class CSVLogger {
public:
  /// @param dir 出力ディレクトリ
  /// @param close_threshold sr.j がこの値に達したら自動 close（<0 で無効）
  /// @param sync_every_n N 行ごとに fdatasync（<=0 でも "毎行" 同期します。N>1 の場合は行バッファ同期を間引き）
  CSVLogger(const std::string& dir, int close_threshold = 100000, int sync_every_n = 1);
  ~CSVLogger();

  /// 1行出力（途中終了でも行単位で残したい場合はこの関数を頻繁に呼ぶ）
  void logData();

  /// 明示的に flush & close（メインスレッドから呼ぶ）
  void flushAndClose();

  /// "YYYYMMDD_HHMMSS" のタイムスタンプ
  static std::string makeTimeStamp();

private:
  // ファイルハンドル
  std::ofstream csv_;
  int           fd_{-1};             // 同一ファイルの低レベルFD（fdatasync用）

  // 状態
  bool          closed_{false};
  int           close_threshold_{100000};
  int           sync_every_n_{1};
  long long     line_counter_{0};

  // シグナル対応
  static std::atomic<bool> sig_close_requested_;
  static void installSignalHandlers();
  static void handleSignal(int signum);
};

#endif // CSV_LOGGER_HPP
