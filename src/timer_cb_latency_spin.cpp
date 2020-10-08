
#include "rclcpp/executors.hpp"
#include "rclcpp/node.hpp"
#include "rclcpp/rclcpp.hpp"
#include "ros2_timer_callback_latency_measurement/util.hpp"
#include "rttest/rttest.h"
#include "rttest/utils.hpp"
#include <bits/stdint-uintn.h>
#include <chrono>
#include <getopt.h> // for getopt_long
#include <limits.h>
#include <rclcpp/strategies/allocator_memory_strategy.hpp>
#include <rclcpp/strategies/message_pool_memory_strategy.hpp>
#include <sched.h>
#include <type_traits>
#include <unistd.h>
#include <unistd.h> // for getopt

#include <malloc.h>
#include <sys/mman.h>

using namespace std::chrono_literals;
using namespace std;
using namespace std::chrono;

struct Params {
  rttest_params rt;
  int main_priority;
  int dds_priority;
  int signal_handler_priority;
  string cb_hist_filename;
  string cb_topn_filename;
  string cb_timeseries_filename;
};

Params get_params(int argc, char *argv[]);

class Timer : public rclcpp::Node {
public:
  explicit Timer(const rclcpp::NodeOptions &options, nanoseconds ns)
      : Node("timer", options) {
    auto callback = [this]() -> void {
      count_++;
      if (rcl_timer_get_time_since_last_call(&(*timer_->get_timer_handle()),
                                             &latency_) == RCL_RET_OK) {
        if (cbHist_) {
          cbHist_->add(latency_);
        }
        if (cbTimeSeries_) {
          cbTimeSeries_->add(latency_);
        }
      }
      if (count_ > count_max_) {
        raise(SIGINT);
      }
    };

    timer_ = create_wall_timer(ns, callback);
  }

  HistReport *cbHist_;
  TimeSeriesReport *cbTimeSeries_;
  int count_ = 0;
  int count_max_;

 private:
   int64_t latency_;
   rclcpp::TimerBase::SharedPtr timer_;
};

int sched_setpriority(const pthread_t &thread, int priority){
  if (priority < 0 || 98 < priority) {
    std::cerr << "priority is wrong." << std::endl;
    return -1;
  }
  struct sched_param param;
  param.sched_priority = priority;
  return sched_setscheduler(thread, SCHED_RR, &param);
}

int main(int argc, char *argv[]) {
  Params params = get_params(argc, argv);

  if (sched_setpriority(0, params.signal_handler_priority)) {
    perror("Couldn't set scheduling priority and policy");
    // return -1;
  }

  // 1 thread created. child thread inheritaneces parent thread sched params.
  // this thread is signal handler.
  rclcpp::init(argc, argv);

  // define executor
  auto exec = std::make_shared<rclcpp::executors::SingleThreadedExecutor>();

  if (sched_setpriority(0, params.dds_priority)) {
    perror("Couldn't set scheduling priority and policy");
    // return -1;
  }
  // multi threads created. child threads inheritaneces parent thread sched
  // params.
  rclcpp::NodeOptions options;
  auto node = make_shared<Timer>(options, toChronoDuration(params.rt.update_period));
  exec->add_node(node);

  if (sched_setpriority(0, params.main_priority)) {
    perror("Couldn't set scheduling priority and policy");
    // return -1;
  }

  if (rttest_lock_and_prefault_dynamic() != 0) {
    fprintf(stderr, "Couldn't lock all cached virtual memory. errno = %d \n", errno);
    fprintf(stderr, "Pagefaults from reading pages not yet mapped into RAM "
                    "will be recorded.\n");
  }

  if (!params.cb_hist_filename.empty() || !params.cb_topn_filename.empty()) {
    node->cbHist_ = new HistReport(1000);
  }
  if (!params.cb_timeseries_filename.empty()) {
    node->cbTimeSeries_ = new TimeSeriesReport(params.rt.iterations);
  }

  node->count_max_ = params.rt.iterations;
  exec->spin();

  // export to csv files.
  if (!params.cb_hist_filename.empty() ) {
    node->cbHist_->histToCsv(params.cb_hist_filename);
  }
  if (!params.cb_topn_filename.empty()) {
    node->cbHist_->topnToHist(params.cb_topn_filename);
  }
  if (!params.cb_timeseries_filename.empty()) {
    node->cbTimeSeries_->toCsv(params.cb_timeseries_filename);
  }

  rclcpp::shutdown();
  return 0;
}

Params get_params(int argc, char *argv[]) {
  Params params;

  const struct option longopts[] = {
      // {name                     ,  has_arg,           flag,            val},
      {"main-priority", required_argument, 0, 'm'},
      {"dds-priority", required_argument, 0, 'd'},
      {"sig-handler-priority", required_argument, 0, 's'},
      {"hist_filename", required_argument, 0, 'h'},
      {"topn_filename", required_argument, 0, 'n'},
      {"timeseries_filename", required_argument, 0, 't'},
      {0, 0, 0, 0},
  };

  opterr = 0;
  optind = 1;
  int longindex = 0;
  int c;
  params.main_priority = -1;
  params.dds_priority = -1;
  params.signal_handler_priority = -1;

  // parse arguments until undefined opt is found such as --rttest_args.
  const std::string optstring = "+";
  while ((c = getopt_long(argc, argv, optstring.c_str(), longopts,
                          &longindex)) != -1) {
    switch (c) {
    case ('m'):
      params.main_priority = std::stoi(optarg);
      std::cout  <<"main " << params.main_priority << std::endl;
      break;
    case ('d'):
      params.dds_priority = std::stoi(optarg);
      std::cout  <<"dds " << params.dds_priority << std::endl;
      break;
    case ('s'):
      params.signal_handler_priority = std::stoi(optarg);
      std::cout << "sig " << params.signal_handler_priority << std::endl;
      break;
    case ('h'):
      params.cb_hist_filename = optarg;
      break;
    case ('n'):
      params.cb_topn_filename = optarg;
      break;
    case ('t'):
      params.cb_timeseries_filename = optarg;
      break;
    }
  }

  // forward argv and argc next to --rttest_args.
  argc -= optind - 2;
  argv += optind - 2;

  if (rttest_read_args(argc, argv) != 0) {
    perror("Couldn't read arguments for rttest");
  }
  rttest_get_params(&params.rt);
  
  return params;
}
