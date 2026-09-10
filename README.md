# Project 4 - PCB and Memory Simulator

This project is a CPU scheduling and segmented memory management simulator. It simulates loading processes into memory, translating logical to physical addresses, and executing processes using a time-sliced round-robin scheduler.

## Files
- `Project4.cpp`: The main C++ source code for the simulator.
- `input.txt`: provided input file containing system parameters and process details to test the program.
- `output.txt` / `my_output.txt`: output files used for comparison against the provided correct output.

## How to Compile and Run

Compile the program using `g++`:
```bash
g++ Project4.cpp -o sim
```

Run the program by piping in an input file:
```bash
./sim < input.txt
```

To save the output to a file:
```bash
./sim < input.txt > my_output.txt
```

## How It Works
1. **Initialization:** Reads total memory, time slice, context switch time, and process definitions from standard input.
2. **Memory Allocation:** Uses segmented memory management to allocate space for process control blocks (PCBs) and instructions. If memory is fragmented, it attempts to coalesce free blocks.
3. **Execution:** Moves processes between the Ready Queue, I/O Waiting Queue, and CPU. It translates logical addresses to physical addresses on the fly.
4. **Instructions:** Supports basic opcodes:
   - `1`: Compute (uses CPU cycles)
   - `2`: I/O (moves to I/O wait queue)
   - `3`: Store value to memory
   - `4`: Load value from memory
5. **Termination:** When a process finishes its instructions, its memory is freed and the simulator moves to the next process until all are complete.

## What is a PCB?
A PCB (Process Control Block) is a data structure used by the operating system to keep track of a running program (a process). It stores all the crucial information the OS needs to manage that process, such as:
- **Process State:** Is it running, waiting for I/O, or ready to run?
- **Program Counter:** Which instruction is it currently on?
- **Registers:** Temporary values it was working with before getting paused.
- **Memory Information:** Where in main memory are its instructions and data stored?

## Why are PCBs important?
PCBs make multitasking possible. Because the CPU can only do one thing at a time, it has to rapidly switch between different processes (called a context switch) to make it look like they are all running at the same time. The PCB acts as a "save state" for a process. When the OS pauses a process, it saves everything into the PCB. When it's time for that process to run again, the OS loads everything back from the PCB and resumes right where it left off.
