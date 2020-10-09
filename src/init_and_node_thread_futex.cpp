#include "rclcpp/executor.hpp"
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
#include <sched.h>
#include <type_traits>
#include <unistd.h>
#include <linux/futex.h>
#include <sys/syscall.h>
#include <sys/time.h>

#include <malloc.h>
#include <sys/mman.h>

#define TRUE 1
#define FALSE 0

using namespace std::chrono_literals;
using namespace std;

struct Params {
  rttest_params rt;
  int main_priority;
  int dds_priority;
  int signal_handler_priority;
  string hist_filename;
  string topn_filename;
  string timeseries_filename;
};

int mem;

Params get_params(int argc, char *argv[]);

int sched_setpriority(const pthread_t &thread, int priority) {
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
  rclcpp::init(argc, argv);

  if (sched_setpriority(0, params.dds_priority)) {
    perror("Couldn't set scheduling priority and policy");
    // return -1;
  }
  // multi threads created. child threads inheritaneces parent thread sched params.
  auto node = rclcpp::Node::make_shared("sample_node");

  if (sched_setpriority(0, params.main_priority)) {
    perror("Couldn't set scheduling priority and policy");
    // return -1;
  }

  HistReport *hist = nullptr;
  HistReport *hist_mono = nullptr;
  TimeSeriesReport *timeSeries = nullptr;
  if (!params.hist_filename.empty() || !params.timeseries_filename.empty()) {
    hist = new HistReport(1000);
    hist_mono = new HistReport(1000);
  }
  if (!params.timeseries_filename.empty()) {
    timeSeries = new TimeSeriesReport(params.rt.iterations);
  }

  struct timespec expected, wakeup, wakeup_latency;
  struct timespec expected_mono, wakeup_mono, wakeup_latency_mono;

  if (rttest_lock_and_prefault_dynamic() != 0) {
    fprintf(stderr, "Couldn't lock all cached virtual memory. errno = %d \n",
            errno);
    fprintf(stderr, "Pagefaults from reading pages not yet mapped into RAM "
                    "will be recorded.\n");
  }

  uint64_t latency;
  uint64_t latency_mono;
  clock_gettime(CLOCK_REALTIME, &expected);
  clock_gettime(CLOCK_MONOTONIC, &expected_mono);

  for (unsigned long i = 0; i < params.rt.iterations; i++) {
    add_timespecs(&expected, &params.rt.update_period, &expected);
    // add_timespecs(&expected_mono, &params.rt.update_period, &expected_mono);

	syscall(SYS_futex, &mem, FUTEX_WAIT_BITSET_PRIVATE | FUTEX_CLOCK_REALTIME, mem, 
			&expected, nullptr, FUTEX_BITSET_MATCH_ANY);

    clock_gettime(CLOCK_REALTIME, &wakeup);
	// clock_gettime(CLOCK_MONOTONIC, &wakeup_mono);

    subtract_timespecs(&wakeup, &expected, &wakeup_latency);
    // subtract_timespecs(&wakeup_mono, &expected_mono, &wakeup_latency_mono);
    latency = timespec_to_uint64(&wakeup_latency);
    // latency_mono = timespec_to_uint64(&wakeup_latency_mono);

    if (hist) {
      hist->add(latency);
      hist_mono->add(latency_mono);
    }
    if (timeSeries) {
      timeSeries->add(latency);
    }
  }

  // export to csv files.
  if( !params.hist_filename.empty() ) {
    hist->histToCsv(params.hist_filename);
    // hist_mono->histToCsv(params.hist_filename + ".mono");
  }
  if( !params.topn_filename.empty() ) {
    hist->topnToHist(params.topn_filename);
    // hist_mono->topnToHist(params.topn_filename + ".mono");
  }
  if( !params.timeseries_filename.empty() ) {
    timeSeries->toCsv(params.timeseries_filename);
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
      std::cout << "main " << params.main_priority << std::endl;
      break;
    case ('d'):
      params.dds_priority = std::stoi(optarg);
      std::cout << "dds " << params.dds_priority << std::endl;
      break;
    case ('s'):
      params.signal_handler_priority = std::stoi(optarg);
      std::cout << "sig " << params.signal_handler_priority << std::endl;
      break;
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

  // forward argv and argc next to --rttest_args.
  argc -= optind - 2;
  argv += optind - 2;

  if (rttest_read_args(argc, argv) != 0) {
    perror("Couldn't read arguments for rttest");
  }
  rttest_get_params(&params.rt);
  return params;
}
