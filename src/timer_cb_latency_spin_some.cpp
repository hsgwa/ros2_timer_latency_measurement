
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
  auto node = rclcpp::Node::make_shared("sample_node");
  exec->add_node(node);
  struct timespec  cb_wakeup, cb_wakeup_latency;
  vector<uint64_t> wakeup_latencies(params.rt.iterations);

  auto timer = node->create_wall_timer(
      toChronoDuration(params.rt.update_period) * 0.5, // every spin_some calls callback.
      [&]() { clock_gettime(CLOCK_MONOTONIC, &cb_wakeup); });

  if (!params.realtime_child && rttest_set_thread_default_priority()) {
    perror("Couldn't set scheduling priority and policy");
    // return -1;
  }

  HistReport *cbHist = nullptr;
  TimeSeriesReport *cbTimeSeries = nullptr;
  if (!params.cb_hist_filename.empty() || !params.cb_topn_filename.empty()) {
    cbHist = new HistReport(1000);
  }

  if (rttest_lock_and_prefault_dynamic() != 0) {
    fprintf(stderr, "Couldn't lock all cached virtual memory. errno = %d \n", errno);
    fprintf(stderr, "Pagefaults from reading pages not yet mapped into RAM "
                    "will be recorded.\n");
  }

  uint64_t latency;
  struct timespec expected;
  clock_gettime(CLOCK_MONOTONIC, &expected);

  for (unsigned long i = 0; i < params.rt.iterations; i++) {
    add_timespecs(&expected, &params.rt.update_period, &expected);

    exec->spin_some();

    subtract_timespecs(&cb_wakeup, &expected, &cb_wakeup_latency);
    latency = timespec_to_long(&cb_wakeup_latency);
    if( cbHist ) {
      cbHist->add(latency);
    }
    if( cbTimeSeries ) {
      cbTimeSeries->add(latency);
    }
  }

  // export to csv files.

  if( !params.cb_hist_filename.empty() ) {
    cbHist->histToCsv(params.cb_hist_filename);
  }
  if( !params.cb_hist_filename.empty() ) {
    cbHist->topnToHist(params.cb_topn_filename);
  }
  if( !params.cb_timeseries_filename.empty() ) {
    cbTimeSeries->toCsv(params.cb_timeseries_filename);
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
      {"wakeup_hist_filename",        required_argument, 0,               'h'},
      {"wakeup_topn_filename",        required_argument, 0,               'n'},
      {"wakeup_timeseries_filename",  required_argument, 0,               't'},
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
