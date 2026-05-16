# OS-Traffic-Parking-Simulator
A feature-rich Operating Systems project simulating dual intersections (F10 &amp; F11) with 15 pthreads, semaphore-based parking management, priority scheduling (Emergency -> Bus -> Normal), mutex/condvar traffic lights, bidirectional IPC pipes, and fork() controllers.  Built with raylib for real-time visualization.

## Features

- Dual intersections (F10 & F11) with independent traffic light control
- 15 vehicle threads with random types and behaviors
- Priority-based scheduling: Emergency > Bus > Normal
- Primary parking lot at F10 with overflow at F11
- Semaphore-managed parking spots and queues
- Mutex and condition variables for traffic synchronization
- Bidirectional IPC pipes between controller processes
- fork() for creating dedicated intersection controllers
- Real-time visualization using raylib

## OS Concepts Implemented

- Multithreading (pthreads)
- Process creation (fork)
- Mutex locks
- Condition variables
- Semaphores
- Inter-process communication (IPC)
- Signal handling (SIGINT)

## Requirements

```bash
sudo apt install build-essential libraylib-dev
