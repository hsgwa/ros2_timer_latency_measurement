
#include "rclcpp/executors.hpp"
#include "rclcpp/node.hpp"
#include "rclcpp/rclcpp.hpp"
#include "ros2_timer_callback_latency_measurement/util.hpp"
#include "rttest/rttest.h"
#include "rttest/utils.h"
#include <bits/stdint-uintn.h>
#include <chrono>
#include <getopt.h> // for getopt_long
#include <limits.h>
#include <rclcpp/strategies/allocator_memory_strategy.hpp>
#include <rclcpp/strategies/message_pool_memory_strategy.hpp>
#include <sched.h>
#include <tlsf_cpp/tlsf.hpp>
#include <type_traits>
#include <unistd.h>
#include <unistd.h> // for getopt

#include <malloc.h>
#include <sys/mman.h>

using namespace std::chrono_literals;
using namespace std;
using namespace std::chrono;

using rclcpp::memory_strategies::allocator_memory_strategy:: AllocatorMemoryStrategy;

struct Params {
  rttest_params rt;
  bool realtime_child;
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

template <typename T = void> using TLSFAllocator = tlsf_heap_allocator<T>;
int main(int argc, char *argv[]) {
  Params params = get_params(argc, argv);

  if (params.realtime_child && rttest_set_thread_default_priority()) {
    perror("Couldn't set scheduling priority and policy");
    // return -1;
  }

  // 1 thread created. child thread inheritaneces parent thread sched params.
  rclcpp::init(argc, argv);

  // define executor
  rclcpp::executor::ExecutorArgs args;
  rclcpp::memory_strategy::MemoryStrategy::SharedPtr memory_strategy =
      std::make_shared<AllocatorMemoryStrategy<TLSFAllocator<void>>>();
  args.memory_strategy = memory_strategy;
  auto exec = std::make_shared<rclcpp::executors::SingleThreadedExecutor>(args);

  // multi threads created. child threads inheritaneces parent thread sched params.
  // FastRTPS    : 5 threads / node
  // CyclondeDDS : 7 threads
  rclcpp::NodeOptions options;
  auto node = make_shared<Timer>(options, toChronoDuration(params.rt.update_period));
  exec->add_node(node);

  if (!params.realtime_child && rttest_set_thread_default_priority()) {
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
  int realtime_child = 0; // default: non-realtime

  const struct option longopts[] = {
      // {name                     ,  has_arg,           flag,            val},
      {"use_realtime_child_thread",   no_argument,       &realtime_child, 1},
      {"unuse_realtime_child_thread", no_argument,       &realtime_child, 0},
      {"cb_hist_filename",            required_argument, 0,               'h'},
      {"cb_topn_filename",            required_argument, 0,               'n'},
      {"cb_timeseries_filename",      required_argument, 0,               't'},
      {0,                             0,                 0,               0},
  };

  opterr = 0;
  optind = 1;
  int longindex = 0;
  int c;

  // parse arguments until undefined opt is found such as --rttest_args.
  const std::string optstring = "+";
  while ((c = getopt_long(argc, argv, optstring.c_str(), longopts,
                          &longindex)) != -1) {
    switch (c) {
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

  params.realtime_child = realtime_child;

  // forward argv and argc next to --rttest_args.
  argc -= optind - 2;
  argv += optind - 2;

  if (rttest_read_args(argc, argv) != 0) {
    perror("Couldn't read arguments for rttest");
  }
  rttest_get_params(&params.rt);
  
  return params;
}
