#include "ros2_timer_callback_latency_measurement/util.hpp"
#include "rttest/rttest.h"
#include "rttest/utils.hpp"
#include <bits/stdint-uintn.h>
#include <chrono>
#include <getopt.h> // for getopt_long
#include <limits.h>
#include <sched.h>
#include <type_traits>
#include <unistd.h> // for getopt
#include <iostream>
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
  int priority;
  int clock_type;
  string nanosleep;
  string hist_filename;
  string topn_filename;
  string timeseries_filename;
};


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

int mem;

int main(int argc, char *argv[]) {
  Params params = get_params(argc, argv);

  if (sched_setpriority(0, params.priority)) {
    perror("Couldn't set scheduling priority and policy");
    // return -1;
  }

  HistReport *hist = nullptr;
  HistReport *hist_mono = nullptr;
  HistReport *hist_mono_raw = nullptr;
  TimeSeriesReport *timeSeries = nullptr;

  if (!params.hist_filename.empty() || !params.timeseries_filename.empty()) {
    hist = new HistReport(1000);
    hist_mono = new HistReport(1000);
    hist_mono_raw = new HistReport(1000);
  }
  if (!params.timeseries_filename.empty()) {
    timeSeries = new TimeSeriesReport(params.rt.iterations);
  }


  int nanosleep_clock = -1;
  if (params.nanosleep == "realtime" ){
    nanosleep_clock = CLOCK_REALTIME;
    std::cout << "use nanosleep CLOCK_REALTIME" << std::endl;
  } else if (params.nanosleep == "monotonic") {
    nanosleep_clock = CLOCK_MONOTONIC;
    std::cout << "use nanosleep CLOCK_MONOTONIC" << std::endl;
  } else if (params.nanosleep.size() > 0) {
    std::cout << "unknown nanosleep argument. use futex" << std::endl;
  }

  if (rttest_lock_and_prefault_dynamic() != 0) {
    fprintf(stderr, "Couldn't lock all cached virtual memory. errno = %d \n", errno);
    fprintf(stderr, "Pagefaults from reading pages not yet mapped into RAM "
                    "will be recorded.\n");
  }

  uint64_t latency;
  uint64_t latency_mono;
  uint64_t latency_mono_raw;
  struct timespec expected, wakeup, wakeup_latency;
  struct timespec expected_mono, wakeup_mono, wakeup_latency_mono;
  struct timespec expected_mono_raw, wakeup_mono_raw, wakeup_latency_mono_raw;

  clock_gettime(CLOCK_REALTIME, &expected);
  clock_gettime(CLOCK_MONOTONIC, &expected_mono);
  clock_gettime(CLOCK_MONOTONIC_RAW, &expected_mono_raw);

  for (unsigned long i = 0; i < params.rt.iterations; i++) {
    add_timespecs(&expected, &params.rt.update_period, &expected);
    add_timespecs(&expected_mono, &params.rt.update_period, &expected_mono);
    add_timespecs(&expected_mono_raw, &params.rt.update_period, &expected_mono_raw);

    switch (nanosleep_clock) {
    case CLOCK_REALTIME:
      clock_nanosleep(nanosleep_clock, TIMER_ABSTIME, &expected, NULL);
      break;
    case CLOCK_MONOTONIC:
      clock_nanosleep(nanosleep_clock, TIMER_ABSTIME, &expected_mono, NULL);
      break;
    default:
      syscall(SYS_futex, &mem, FUTEX_WAIT_BITSET_PRIVATE | FUTEX_CLOCK_REALTIME,
              mem, &expected, nullptr, FUTEX_BITSET_MATCH_ANY);
    }

    clock_gettime(CLOCK_REALTIME, &wakeup);
    clock_gettime(CLOCK_MONOTONIC, &wakeup_mono);
    clock_gettime(CLOCK_MONOTONIC_RAW, &wakeup_mono_raw);

    subtract_timespecs(&wakeup, &expected, &wakeup_latency);
    subtract_timespecs(&wakeup_mono, &expected_mono, &wakeup_latency_mono);
    subtract_timespecs(&wakeup_mono_raw, &expected_mono_raw,
                       &wakeup_latency_mono_raw);

    latency = timespec_to_uint64(&wakeup_latency);
    latency_mono = timespec_to_uint64(&wakeup_latency_mono);
    latency_mono_raw = timespec_to_uint64(&wakeup_latency_mono_raw);

    if (hist) {
      hist->add(latency);
      hist_mono->add(latency_mono);
      hist_mono_raw->add(latency_mono_raw);
    }
    if (timeSeries) {
      timeSeries->add(latency);
    }
  }

  // export to csv files.
  if( !params.hist_filename.empty() ) {
    hist->histToCsv(params.hist_filename);
    hist_mono->histToCsv(params.hist_filename + ".mono");
    hist_mono_raw->histToCsv(params.hist_filename + ".mono_raw");
  }
  if( !params.topn_filename.empty() ) {
    hist->topnToHist(params.topn_filename);
  }
  if( !params.timeseries_filename.empty() ) {
    timeSeries->toCsv(params.timeseries_filename);
  }

  return 0;
}


Params get_params(int argc, char *argv[]) {
  Params params;

  const struct option longopts[] = {
      // {name                     ,  has_arg,           flag,            val},
      {"priority", required_argument, 0, 'p'},
      {"hist_filename", required_argument, 0, 'h'},
      {"topn_filename", required_argument, 0, 'n'},
      {"timeseries_filename", required_argument, 0, 't'},
      {"nanosleep", required_argument, 0, 's'},
      {0, 0, 0, 0},
  };

  opterr = 0;
  optind = 1;
  int longindex = 0;
  int c;
  params.priority = -1;

  // parse arguments until undefined opt is found such as --rttest_args.
  const std::string optstring = "+";
  while ((c = getopt_long(argc, argv, optstring.c_str(), longopts,
                          &longindex)) != -1) {
    switch (c) {
    case ('p'):
      params.priority = std::stoi(optarg);
      std::cout << "priority " << params.priority << std::endl;
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
    case ('s'):
      params.nanosleep = optarg;
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
