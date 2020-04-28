#include "rclcpp/executor.hpp"
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
  string hist_filename;
  string topn_filename;
  string timeseries_filename;
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
  if (!params.realtime_child && rttest_set_thread_default_priority()) {
    perror("Couldn't set scheduling priority and policy");
    // return -1;
  }

  HistReport *hist = nullptr;
  TimeSeriesReport *timeSeries =nullptr;
  if ( !params.hist_filename.empty() || !params.timeseries_filename.empty()) {
    hist = new HistReport(100);
  }
  if (!params.timeseries_filename.empty()) {
    timeSeries = new TimeSeriesReport(params.rt.iterations);
  }

  struct timespec expected, wakeup, wakeup_latency;
  vector<uint64_t> wakeup_latencies(params.rt.iterations);

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

    subtract_timespecs(&wakeup, &expected, &wakeup_latency);

    latency = timespec_to_long(&wakeup_latency);
    if (hist) {
      hist->add(latency);
    }
    if (timeSeries) {
      timeSeries->add(latency);
    }
  }

  if ( !params.hist_filename.empty() ) {
    hist->saveHist(params.hist_filename);
  }
  if ( !params.topn_filename.empty() ) {
    hist->saveTopN(params.topn_filename);
  }
  if ( !params.timeseries_filename.empty() ) {
    timeSeries->save(params.timeseries_filename);
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
      {"hist_filename", required_argument, 0, 'h'},
      {"topn_filename", required_argument, 0, 'n'},
      {"timeseries_filename", required_argument, 0, 't'},
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
      params.hist_filename = optarg;
      break;
    case ('n'):
      params.topn_filename = optarg;
      break;
    case ('t'):
      params.timeseries_filename = optarg;
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
