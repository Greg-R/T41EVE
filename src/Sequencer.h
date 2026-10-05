/*
T41EVE Copyright 2026 Gregory Raven

This file is part of T41EVE.

T41EVE is free software: you can redistribute it and/or modify it under the terms of the GNU General Public License as published by the Free Software Foundation, either version 3 of the License, or (at your option) any later version.

T41EVE is distributed in the hope that it will be useful, but WITHOUT ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the GNU General Public License for more details.

You should have received a copy of the GNU General Public License along with T41EVE. If not, see <https://www.gnu.org/licenses/>.

  This comment block must appear in the load page (e.g., main() or setup()) in any source code
  that uses code presented as whole or part of the T41-EP source code.

  (c) Frank Dziock, DD4WH, 2020_05_8
  "TEENSY CONVOLUTION SDR" substantially modified by Jack Purdum, W8TEE, and Al Peter, AC8GY

  This software is made available under the GNU GPLv3 license agreement. If commercial use of this
  software is planned, we would appreciate it if the interested parties contact Jack Purdum, W8TEE, 
  and Al Peter, AC8GY.

  Any and all other uses, written or implied, by the GPLv3 license are forbidden without written 
  permission from from Jack Purdum, W8TEE, and Al Peter, AC8GY.
*/

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
      if (pq.size() < 20)
      if(sequencerActive)  pq.push(task);
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
    static int size()
    {
      return static_cast<int>(pq.size());
    }

    static void active(bool set) {
      sequencerActive = set;
    }

  private:
    // Priority queue.
    static inline std::priority_queue<Task, std::vector<Task>> pq{};
    static inline bool sequencerActive{true};

  }; // End class sequencer

  using task_seq = Sequencer::task<callable_holder>;
  using seq = Sequencer::sequencer<task_seq>;

} // End Sequencer namespace.
