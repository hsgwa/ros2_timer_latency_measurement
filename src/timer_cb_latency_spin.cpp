#include "rcl/timer.h"
#include "rclcpp/executors.hpp"
#include "rclcpp/node.hpp"
#include "rttest/rttest.h"
#include "rttest/utils.h"
#include <bits/stdint-uintn.h>
#include <chrono>
#include <errno.h>
#include <limits.h>
#include <rclcpp/strategies/allocator_memory_strategy.hpp>
#include <rclcpp/strategies/message_pool_memory_strategy.hpp>
#include <sched.h>
#include <signal.h>
#include <stdio.h>
#include <string.h>
#include <sys/resource.h>
#include <sys/time.h>
#include <tlsf_cpp/tlsf.hpp>
#include <type_traits>
#include <unistd.h>

#include <malloc.h>
#include <sys/mman.h>


using rclcpp::memory_strategies::allocator_memory_strategy::
    AllocatorMemoryStrategy;

template <typename T = void> using TLSFAllocator = tlsf_heap_allocator<T>;

void print_rusage(const struct rusage &before, const struct rusage &after);

using namespace std;
using namespace std::chrono_literals;

class Timer : public rclcpp::Node {
public:
  explicit Timer(const rclcpp::NodeOptions &options) : Node("timer", options) {
    auto callback = [this]() -> void {
      count_++;
      if (rcl_timer_get_time_since_last_call(
              &(*timer_->get_timer_handle()), &latency_) == RCL_RET_OK) {
        wakeup_latencies_[count_] = latency_;
      }
    };

    auto kill = []() -> void { raise(SIGINT); };

    timer_ = create_wall_timer(10ms, callback);
    timer_kill_ = create_wall_timer(2h, kill);
  }
  int64_t latency_;
  int64_t wakeup_latencies_[2000000];
  int count_ = 0;
  rclcpp::TimerBase::SharedPtr timer_;
  rclcpp::TimerBase::SharedPtr timer_kill_;
};

int main(int argc, char *argv[])
{

  rclcpp::init(argc, argv);
  rclcpp::NodeOptions options;
  rclcpp::memory_strategy::MemoryStrategy::SharedPtr memory_strategy =
      std::make_shared<AllocatorMemoryStrategy<TLSFAllocator<void>>>();

  rclcpp::executor::ExecutorArgs args;
  args.memory_strategy = memory_strategy;
  auto exec = std::make_shared<rclcpp::executors::SingleThreadedExecutor>(args);
  auto node = std::make_shared<Timer>(options);
  exec->add_node(node);

  if (rttest_set_sched_priority(98, SCHED_RR)) {
    perror("Couldn't set scheduling priority and policy");
  }

  if (rttest_lock_and_prefault_dynamic() != 0) {
    fprintf(stderr, "Couldn't lock all cached virtual memory. errno = %d \n", errno);
    fprintf(stderr, "Pagefaults from reading pages not yet mapped into RAM "
                    "will be recorded.\n");
  }


  struct rusage usage_start, usage_end;

  if (getrusage(RUSAGE_SELF, &usage_start) != 0) {
    perror("Couldn't get rusage");
  }

  exec->spin();

  if (getrusage(RUSAGE_SELF, &usage_end) != 0) {
    perror("Couldn't get rusage");
  }
  print_rusage(usage_start, usage_end);

  for (int i = 1; i < node->count_; i++) {
    printf("%d,%llu \n", i, node->wakeup_latencies_[i]);
  }
  rclcpp::shutdown();
  return 0;
}

void print_rusage(const struct rusage &before, const struct rusage &after)
{
  cerr << "minor page fault : ";
  cerr << after.ru_minflt - before.ru_minflt << endl;

  cerr << "major page fault : ";
  cerr << after.ru_majflt - before.ru_majflt << endl;

  cerr << "swap : ";
  cerr << after.ru_nswap - before.ru_nswap << endl;

  cerr << "voluntary context switches : ";
  cerr << after.ru_nvcsw - before.ru_nvcsw << endl;

  cerr << "involuntary context switches : ";
  cerr << after.ru_nivcsw - before.ru_nivcsw << endl;
}
