#include <string>
#include <vector>
#include <chrono>

class Hist;
class TimeSeries;


class HistReport {
 public:
  HistReport(int bins_num, uint64_t bin_width_ns=1000, int max_topn=10);
  void add(uint64_t ns);
  void histToCsv(std::string filename);
  void topnToHist(std::string filename);

private:
  inline int getNextIdx(const int &i);
  inline int getPrevIdx(const int &i);
  std::vector<unsigned int> bins_;
  int bins_num_;
  uint64_t bin_width_ns_;

  int idx_;
  int max_topn_;
  int head_idx_;
  std::vector<uint64_t> top_ns_;
  std::vector<int> top_idx_;
};

class TimeSeriesReport {
 public:
   TimeSeriesReport(int max_data_num);
   void add(uint64_t ns);
   void toCsv(std::string filename);

 private:
   int max_data_num_;
   int idx_;
   std::vector<uint64_t> data_;
};

std::chrono::nanoseconds toChronoDuration(timespec ts);
