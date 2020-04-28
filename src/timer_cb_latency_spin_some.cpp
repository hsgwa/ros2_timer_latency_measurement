
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

#define TRUE 1
#define FALSE 0

using namespace std::chrono_literals;
using namespace std;

using rclcpp::memory_strategies::allocator_memory_strategy:: AllocatorMemoryStrategy;

struct Params {
  rttest_params rt;
  bool realtime_child;
  string wakeup_hist_filename;
  string wakeup_topn_filename;
  string wakeup_timeseries_filename;
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
  rclcpp::init(argc, argv);

  rclcpp::executor::ExecutorArgs args;
  rclcpp::memory_strategy::MemoryStrategy::SharedPtr memory_strategy =
      std::make_shared<AllocatorMemoryStrategy<TLSFAllocator<void>>>();
  args.memory_strategy = memory_strategy;
  auto exec = std::make_shared<rclcpp::executors::SingleThreadedExecutor>(args);
  auto node = rclcpp::Node::make_shared("sample_node");
  exec->add_node(node);
  if (!params.realtime_child && rttest_set_thread_default_priority()) {
    perror("Couldn't set scheduling priority and policy");
    // return -1;
  }

  HistReport *wakeupHist = nullptr;
  TimeSeriesReport *wakeupTimeSeries = nullptr;
  if (!params.wakeup_hist_filename.empty() || !params.wakeup_topn_filename.empty()) {
    wakeupHist = new HistReport(100);
  }
  if (!params.wakeup_timeseries_filename.empty()) {
    wakeupTimeSeries = new TimeSeriesReport(params.rt.iterations);
  }

  HistReport *cbHist = nullptr;
  TimeSeriesReport *cbTimeSeries = nullptr;
  if (!params.cb_hist_filename.empty() || !params.cb_topn_filename.empty()) {
    cbHist = new HistReport(100);
  }
  if (!params.wakeup_timeseries_filename.empty()) {
    cbTimeSeries = new TimeSeriesReport(params.rt.iterations);
  }

  struct timespec expected, wakeup, wakeup_latency, cb_wakeup, cb_wakeup_latency;
  vector<uint64_t> wakeup_latencies(params.rt.iterations);
  auto timer = node->create_wall_timer(
      5ms, [&]() { clock_gettime(CLOCK_MONOTONIC, &cb_wakeup); });

  if (rttest_lock_and_prefault_dynamic() != 0) {
    fprintf(stderr, "Couldn't lock all cached virtual memory. errno = %d \n",
            errno);
    fprintf(stderr, "Pagefaults from reading pages not yet mapped into RAM "
                    "will be recorded.\n");
  }

  uint64_t latency;
  clock_gettime(CLOCK_MONOTONIC, &expected);

  for (unsigned long i = 0; i < params.rt.iterations; i++) {
    add_timespecs(&expected, &params.rt.update_period, &expected);

    clock_nanosleep(CLOCK_MONOTONIC, TIMER_ABSTIME, &expected, NULL);
    clock_gettime(CLOCK_MONOTONIC, &wakeup);

    exec->spin_some();

    subtract_timespecs(&wakeup, &expected, &wakeup_latency);
    latency = timespec_to_long(&wakeup_latency);
    if( wakeupHist ) {
      wakeupHist->add(latency);
    }
    if( wakeupTimeSeries ) {
      wakeupTimeSeries->add(latency);
    }

    subtract_timespecs(&cb_wakeup, &expected, &cb_wakeup_latency);
    latency = timespec_to_long(&cb_wakeup_latency);
    if( cbHist ) {
      cbHist->add(latency);
    }
    if( cbTimeSeries ) {
      cbTimeSeries->add(latency);
    }
  }

  if( !params.wakeup_hist_filename.empty() ) {
    wakeupHist->saveHist(params.wakeup_hist_filename);
  }
  if( !params.wakeup_topn_filename.empty() ) {
    wakeupHist->saveTopN(params.wakeup_topn_filename);
  }
  if( !params.wakeup_timeseries_filename.empty() ) {
    wakeupTimeSeries->save(params.wakeup_timeseries_filename);
  }

  if( !params.cb_hist_filename.empty() ) {
    cbHist->saveHist(params.cb_hist_filename);
  }
  if( !params.cb_hist_filename.empty() ) {
    cbHist->saveTopN(params.cb_topn_filename);
  }
  if( !params.cb_timeseries_filename.empty() ) {
    cbTimeSeries->save(params.cb_timeseries_filename);
  }

  rclcpp::shutdown();
  return 0;
}

Params get_params(int argc, char *argv[]) {
  Params params;
  int realtime_child = 0; // default: non-realtime

  const struct option longopts[] = {
      {"realtime_child_thread", no_argument, &realtime_child, 1},
      {"non_realtime_child_thread", no_argument, &realtime_child, 0},
      {"wakeup_hist_filename", required_argument, 0, 'h'},
      {"wakeup_topn_filename", required_argument, 0, 'n'},
      {"wakeup_timeseries_filename", required_argument, 0, 't'},
      {"cb_hist_filename", required_argument, 0, 'i'},
      {"cb_topn_filename", required_argument, 0, 'u'},
      {"cb_timeseries_filename", required_argument, 0, 'm'},
      {0, 0, 0, 0},
  };

  opterr = 0;
  optind = 1;
  int longindex = 0;
  const std::string optstring = "+";
  int c;

  while ((c = getopt_long(argc, argv, optstring.c_str(), longopts,
                          &longindex)) != -1) {
    switch (c) {
    case ('h'):
      params.wakeup_hist_filename = optarg;
      break;
    case ('n'):
      params.wakeup_topn_filename = optarg;
      break;
    case ('t'):
      params.wakeup_timeseries_filename = optarg;
      break;
    case ('i'):
      params.cb_hist_filename = optarg;
      break;
    case ('u'):
      params.cb_topn_filename = optarg;
      break;
    case ('m'):
      params.cb_timeseries_filename = optarg;
      break;
    }
  }
  params.realtime_child = realtime_child;

  argc -= optind - 2;
  argv += optind - 2;

  cout << argv[1] << endl;
  if (rttest_read_args(argc, argv) != 0) {
    perror("Couldn't read arguments for rttest");
  }
  rttest_get_params(&params.rt);
  
  return params;
}
