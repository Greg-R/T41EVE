// Sequencer.h

#pragma once

#include <functional>

namespace Sequencer
{

  using callable_holder = std::function<void()>;

  // The task class:
  template <typename CallableHolder>
  class task
  {
  public:
    constexpr static uint8_t c_prio_default = 250;
    constexpr static uint8_t c_prio_max = 255;
    constexpr static uint8_t c_prio_min = 0;

    // Constructor
    task(CallableHolder the_task, uint8_t prio = c_prio_default) : the_task_(the_task), priority_(prio) {}

    // execute(void) in UML.
    void execute()
    {
      if (the_task_)
      {
        the_task_();
      }
    }

    // Less than (<) operator.  This gives the ability to determine if this task is lower priority than a different task.
    bool operator<(const task &rhs) const
    {
      return priority_ < rhs.priority_;
    }

  private:
    CallableHolder the_task_;
    uint8_t priority_ = c_prio_default;

  }; // End class task.

  // The sequencer class:

  template <typename Task>
  struct sequencer
  {

    sequencer() = delete;

    static void add(Task task)
    {
      noInterrupts();
      pq.push(task);
      interrupts();
    }

    static void run()
    {
      if (!pq.empty())
      {
        noInterrupts();
        auto task = pq.top();
        pq.pop();
        interrupts();
        task.execute();
      }
    }

    static void clear()
    {
      while (!pq.empty())
      {
        pq.pop();
      }
    }

  private:
    // Priority queue.
    static inline std::priority_queue<Task, std::vector<Task>> pq{};

  }; // End class sequencer

} // End Sequencer namespace.
