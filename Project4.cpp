#include <iostream>
#include <sstream>
#include <queue>
#include <vector>
#include <list>
#include <unordered_map>
#include <stdexcept>
#include <string>
#include <iterator>    // for std::next

using namespace std;

// --- Constants ---
const int CTRL_SEG_SIZE   = 3;  // Cells 0-2: control segment
const int PCB_FIELDS      = 10; // Cells 3-12: original PCB fields
const int PCB_BLOCK_SIZE  = CTRL_SEG_SIZE + PCB_FIELDS; // = 13
const int EXTRA_MARGIN    = 10; // Extra space for instructions/parameters
const int MAX_SEGMENTS    = 6;  // Maximum number of segments per process

// Process states
enum ProcessState {
    READY     = 1,
    RUNNING   = 2,
    IOWAITING = 3,
    TERMINATED= 4
};

class IOInterrupt : public exception { };
class TimeOUT     : public exception { };

// The input‐side PCB description
struct PCB {
    int processID;
    int maxMemoryNeeded;
    int instrStreamLength;
    int registerValue; // Added registerValue member
    vector<int> tokens; // opcode & parameters
    vector<int> logicalMemory;
};

// For I/O waiting
struct waitingInfo {
    int pcbAddress;  // address of the PCB
    int startTime;   // when it went to wait
    int waitDuration;
};

// Memory block descriptor
struct MemBlock {
    int processID;     // -1 if free
    int startAddress;
    int size;

    // C++98‑style constructor
    MemBlock(int pid, int start, int sz)
      : processID(pid), startAddress(start), size(sz)
    {}

    // default constructor so leftover/reallocation still works
    MemBlock() {}
};
struct tableEntry {
    int startingAddress;
    int size;
  };

// Prints current memory block list
void printMemChunks(const list<MemBlock>& memoryBlocks) {
    cout << "PRINTING MEMORY BLOCKS:\n";
    cout << "---------------------------------\n";
    for (list<MemBlock>::const_iterator it = memoryBlocks.begin();
         it != memoryBlocks.end();
         ++it)
    {
        const MemBlock& block = *it;
        cout << "Process ID: " << block.processID
             << ", Start Address: " << block.startAddress
             << ", Size: " << block.size << "\n";
    }
}

// Translate a logical address within a process to a physical address
int translateLogicalToPhysical(int logicalAddress, int* pcb) {
    int segmentTableSize = pcb[0];
    int numSegments = segmentTableSize / 2;
    int remaining = logicalAddress;

    for (int i = 0; i < numSegments; ++i) {
        int start  = pcb[1 + 2*i];
        int length = pcb[1 + 2*i + 1];
        if (remaining < length) {
            return start + remaining;
        } else {
            remaining -= length;
        }
    }
    // TODO: Update this print to include the processID as required by the PDF:
    // "Memory violation: address <X> out of bounds for Process <PID>"
    cout << "Memory violation: address "
         << logicalAddress
         << " out of bounds for process " << pcb[3] << "\n";
    return -1;
}

// Copy a process's logical memory into its allocated segments
void copyProcessToMemory(int* processLogicalMemory, int totalLogicalSize, int* pcb, int* mainMemory)
{
    int segmentTableSize = pcb[0];
    int numSegments      = segmentTableSize / 2;
    int logicalIndex     = 0;

    for (int i = 0; i < numSegments; ++i) {
        int start = pcb[1 + 2*i];       // physical start of segment i
        int size  = pcb[1 + 2*i + 1];   // size of segment i
        for (int j = 0;
             j < size && logicalIndex < totalLogicalSize;
             ++j, ++logicalIndex)
        {
            mainMemory[start + j] = processLogicalMemory[logicalIndex];
        }
    }

    if (logicalIndex < totalLogicalSize) {
        cout << "Error: not enough space in allocated segments to hold process.\n";
    }
}

// Globals to track per‐PCB state
unordered_map<int,int> paramIndexMap;    // next parameter index
unordered_map<int,int> processStartTime; // when process first ran

// Load as many jobs as will fit into memory, respecting MAX_SEGMENTS
void loadJobsToMemory(queue<PCB>& newJobQueue, queue<int>& readyQueue, vector<int>& mainMemory, int maxMemory, list<MemBlock>& memoryBlocks)
{
    bool allocationHappened = true;

    while (!newJobQueue.empty() && allocationHappened) {
        allocationHappened = false;
        PCB    currentJob   = newJobQueue.front();
        int    requiredSize = currentJob.maxMemoryNeeded + EXTRA_MARGIN + PCB_BLOCK_SIZE;
        int    allocAddress = -1;

        // 1) Gather free blocks up to MAX_SEGMENTS
        vector<list<MemBlock>::iterator> chunks;
        int foundSize = 0;
        for (list<MemBlock>::iterator it = memoryBlocks.begin();
             it != memoryBlocks.end();
             ++it)
        {
            if (it->processID != -1) continue;
            if (chunks.empty() && it->size < PCB_BLOCK_SIZE) continue;
            chunks.push_back(it);
            foundSize += it->size;
            if (foundSize >= requiredSize || chunks.size() == MAX_SEGMENTS)
                break;
        }

        // 2) If enough, carve out space
        vector<int> allocSizes;
        if (foundSize >= requiredSize && chunks.size() <= MAX_SEGMENTS) {
            allocSizes.reserve(chunks.size());
            int toAllocate = requiredSize;

            // determine how much from each chunk
            for (size_t i = 0; i < chunks.size(); ++i) {
                list<MemBlock>::iterator it = chunks[i];
                int used = (it->size < toAllocate ? it->size : toAllocate);
                allocSizes.push_back(used);
                toAllocate -= used;
                if (toAllocate == 0) break;
            }

            allocAddress = chunks.front()->startAddress;
            // split or consume each chunk
            for (size_t i = 0; i < allocSizes.size(); ++i) {
                list<MemBlock>::iterator it = chunks[i];
                int used   = allocSizes[i];
                int remain = it->size - used;
                it->processID = currentJob.processID;
                it->size      = used;
                if (remain > 0) {
                    MemBlock leftover(-1,
                        it->startAddress + used,
                        remain);
                    memoryBlocks.insert(std::next(it), leftover);
                }
            }
            allocationHappened = true;
        }

        if (allocationHappened) {
            // Remove job and write its PCB & code into memory
            newJobQueue.pop();
            int pcbBase      = allocAddress;
            int numSegs      = static_cast<int>(allocSizes.size());
            int segTableSize = numSegs * 2;

            // -- Control segment
            mainMemory[pcbBase + 0] = segTableSize;
            for (int s = 0; s < numSegs; ++s) {
                mainMemory[pcbBase + 1 + 2*s    ] = chunks[s]->startAddress;
                mainMemory[pcbBase + 1 + 2*s + 1] = allocSizes[s];
            }

            // -- Compute instruction/data bases
            int instructionBase = pcbBase + PCB_BLOCK_SIZE;
            int numOpcodes      = currentJob.instrStreamLength;
            int dataBase        = instructionBase + numOpcodes;

            // -- Fill in PCB fields (after control segment)
            int N = segTableSize + 1;
            int increment = 0;
            mainMemory[ translateLogicalToPhysical(N + increment, &mainMemory[pcbBase]) ] = currentJob.processID; ++increment;
            mainMemory[ translateLogicalToPhysical(N + increment, &mainMemory[pcbBase]) ] = READY;                    ++increment;
            mainMemory[ translateLogicalToPhysical(N + increment, &mainMemory[pcbBase]) ] = 0;                        ++increment;
            mainMemory[ translateLogicalToPhysical(N + increment, &mainMemory[pcbBase]) ] = PCB_BLOCK_SIZE;           ++increment;
            mainMemory[ translateLogicalToPhysical(N + increment, &mainMemory[pcbBase]) ] = N + 10 + numOpcodes; ++increment;
            mainMemory[ translateLogicalToPhysical(N + increment, &mainMemory[pcbBase]) ] = currentJob.maxMemoryNeeded;  ++increment;
            mainMemory[ translateLogicalToPhysical(N + increment, &mainMemory[pcbBase]) ] = 0;                        ++increment;
            mainMemory[ translateLogicalToPhysical(N + increment, &mainMemory[pcbBase]) ] = currentJob.registerValue;                        ++increment;
            mainMemory[ translateLogicalToPhysical(N + increment, &mainMemory[pcbBase]) ] = currentJob.maxMemoryNeeded;  ++increment;
            mainMemory[ translateLogicalToPhysical(N + increment, &mainMemory[pcbBase]) ] = pcbBase;                    ++increment;

            // -- Copy opcodes & parameters in
            vector<int> opcodes;
            vector<int> parameters;
            int tokenIndex = 0;
            for (int i = 0; i < numOpcodes && tokenIndex < currentJob.tokens.size(); i++) {
                int opcode = currentJob.tokens[tokenIndex++];
                opcodes.push_back(opcode);
                if (opcode == 1 || opcode == 3) {
                    int param1 = (tokenIndex < currentJob.tokens.size()) ? currentJob.tokens[tokenIndex++] : -1;
                    int param2 = (tokenIndex < currentJob.tokens.size()) ? currentJob.tokens[tokenIndex++] : -1;
                    parameters.push_back(param1);
                    parameters.push_back(param2);
                } else if (opcode == 2 || opcode == 4) {
                    int param = (tokenIndex < currentJob.tokens.size()) ? currentJob.tokens[tokenIndex++] : -1;
                    parameters.push_back(param);
                }
            }

            for (size_t i = 0; i < opcodes.size(); i++) {
                mainMemory[instructionBase + i] = opcodes[i];
                //cout << "Instruction " << i << ": " << opcodes[i] << "\n";
                
            }
            for (size_t i = 0; i < parameters.size(); i++) {
                mainMemory[dataBase + i] = parameters[i];
                //cout << "Parameter " << i << ": " << parameters[i] << "\n";
                
            }

            paramIndexMap[currentJob.processID] = 0;
            readyQueue.push(pcbBase);
            //printQueues(readyQueue, ioWaitingQueue, newJobQueue);
            int* pcb = mainMemory.data() + pcbBase;
            cout << "Process " << currentJob.processID << " loaded with segment table stored at physical address " 
                 << pcbBase << "\n";
                 // TODO: Remove this debug print
                 printMemChunks(memoryBlocks);
                 
        }
        else {
            // TODO: If the memory allocation failed because a single chunk of >= 13 (for the segment table) 
            // could not be found, you must print the EXACT string from the PDF:
            // "Process <PID> could not be loaded due to insufficient contiguous space for segment table."
            // Otherwise, if it's just general memory shortage, print whatever is required (or nothing).
            // coalesce adjacent free blocks and retry
            cout << "Insufficient memory for Process "
                 << newJobQueue.front().processID
                 << ". Attempting memory coalescing.\n";
            for (list<MemBlock>::iterator it = memoryBlocks.begin();
                 it != memoryBlocks.end(); )
            {
                list<MemBlock>::iterator nextIt = it;
                ++nextIt;
                if (nextIt != memoryBlocks.end()
                    && it->processID == -1
                    && nextIt->processID == -1)
                {
                    it->size += nextIt->size;
                    it = memoryBlocks.erase(nextIt);
                }
                else {
                    ++it;
                }
            }
            // see if any single block now fits
            for (list<MemBlock>::iterator blkIt = memoryBlocks.begin();
                 blkIt != memoryBlocks.end();
                 ++blkIt)
            {
                if (blkIt->processID == -1
                    && blkIt->size >= requiredSize)
                {
                    allocationHappened = true;
                    cout << "Memory coalesced. Process "
                         << newJobQueue.front().processID
                         << " can now be loaded.\n";
                    break;
                }
            }
        }
    }

    if (!newJobQueue.empty()) {
        // TODO: Ensure this doesn't conflict with the "insufficient contiguous space for segment table" requirement.
        cout << "Process " << newJobQueue.front().processID
             << " waiting in NewJobQueue due to insufficient memory.\n";
    }
}

// Run CPU on one process until time slice or termination
// ---------------------------------------------------------
// Run CPU on one process until its time slice expires or it
// terminates.  Uses segmented address translation everywhere.
// ---------------------------------------------------------
void executeCPU(int pcbAddress, vector<int>& mainMemory, int CPUAllocated, int contextSwitchTime, int &globalClock, queue<int>& readyQueue, queue<waitingInfo>& ioWaitingQueue, list<MemBlock>& memoryBlocks)
{
// ─── 1.  Recreate segment‑table metadata ─────────────────────────────
int segmentTableSize = mainMemory[pcbAddress];          // logical 0
int offset           = segmentTableSize + 1;            // logical index where PCB fields start

// Logical indices of PCB fields (see project PDF)
const int LOG_PID       = offset + 0;
const int LOG_STATE     = offset + 1;
const int LOG_PC        = offset + 2;
const int LOG_IBASE     = offset + 3;
const int LOG_DBASE     = offset + 4;
const int LOG_MEMLIM    = offset + 5;
const int LOG_CPUUSED   = offset + 6;
const int LOG_REGVAL    = offset + 7;
const int LOG_MAXMEM    = offset + 8;
const int LOG_MAINBASE  = offset + 9;

// Physical addresses of mutable PCB fields
int  physPID      = translateLogicalToPhysical(LOG_PID,    &mainMemory[pcbAddress]);
int  physState    = translateLogicalToPhysical(LOG_STATE,  &mainMemory[pcbAddress]);
int  physPC       = translateLogicalToPhysical(LOG_PC,     &mainMemory[pcbAddress]);
int  physCPUUsed  = translateLogicalToPhysical(LOG_CPUUSED,&mainMemory[pcbAddress]);
int  physRegVal   = translateLogicalToPhysical(LOG_REGVAL, &mainMemory[pcbAddress]);

// Fixed (read‑only) values
int processID     = mainMemory[physPID];
int instrBaseRel  = mainMemory[ translateLogicalToPhysical(LOG_IBASE, &mainMemory[pcbAddress]) ];
int dataBaseRel   = mainMemory[ translateLogicalToPhysical(LOG_DBASE, &mainMemory[pcbAddress]) ];
int memoryLimit   = mainMemory[ translateLogicalToPhysical(LOG_MEMLIM,&mainMemory[pcbAddress]) ];

cout << "Process " << processID << " has moved to Running.\n";

// References to mutable PCB integers
int &state          = mainMemory[physState];
int &programCounter = mainMemory[physPC];
int &cpuCyclesUsed  = mainMemory[physCPUUsed];
int &registerValue  = mainMemory[physRegVal];

int numOpcodes      = dataBaseRel - instrBaseRel;
int &paramIdx       = paramIndexMap[processID];

// TODO: Remove this debug print
// Print the whole paramIndexMap
for( auto it = paramIndexMap.begin(); it != paramIndexMap.end(); ++it ) {
    cout << "Process ID: " << it->first << ", Param Index: " << it->second << "\n";
}

//paramIndexMap AND processStartTime should be by processID

// First‑time start timestamp
if (processStartTime.find(processID) == processStartTime.end())
processStartTime[processID] = globalClock;

// ─── 2.  Execute until slice exhausted or process ends ───────────────
int localCycles = 0;

while (programCounter < numOpcodes && localCycles < CPUAllocated)
{
// TODO: The PDF asks to print "Logical address <X> translated to physical address <Y> for Process <PID>" 
// "When a logical address is translated". You may need to add that print statement here for instruction fetches.
int physInstr = translateLogicalToPhysical(instrBaseRel + programCounter,&mainMemory[pcbAddress]);
int opcode    = mainMemory[physInstr];



vector<int> params;
if (opcode == 1 || opcode == 3) {             // two‑parameter op
int a = translateLogicalToPhysical(dataBaseRel + paramIdx, &mainMemory[pcbAddress]);
int b = translateLogicalToPhysical(dataBaseRel + paramIdx + 1, &mainMemory[pcbAddress]);
params.push_back(mainMemory[a]);
params.push_back(mainMemory[b]);
paramIdx += 2;
}
else if (opcode == 2 || opcode == 4) {        // one‑parameter op
int a = translateLogicalToPhysical(dataBaseRel + paramIdx, &mainMemory[pcbAddress]);
params.push_back(mainMemory[a]);
paramIdx++;
}
else {
cerr << "ERROR: Invalid opcode " << opcode << "\n";
++programCounter;
continue;
}

// -------- Opcode semantics --------------------------------------
switch (opcode)
{
case 1: {  // compute (busy‑wait)
int cycles   = (params.size() >= 2) ? params[1] : 0;
cpuCyclesUsed += cycles;
globalClock   += cycles;
localCycles   += cycles;
// TODO: Remove this debug print
cout << "compute\n";
break;
}
case 2: {  // I/O – move to waiting queue
int wait = (params.size() >= 1) ? params[0] : 0;
cout << "Process " << processID
     << " issued an IOInterrupt and moved to the IOWaitingQueue.\n";
cpuCyclesUsed += wait;
waitingInfo wi = { pcbAddress, globalClock, wait };
ioWaitingQueue.push(wi);
++programCounter;
return;                                   // slice ends on I/O
}
case 3: {  // store
// TODO: Remove this debug print
for(int i = 0; i < params.size(); i++)
{
    cout << params[i] << " ";
}
int value       = (params.size() >= 1) ? params[0] : -1;
int logicalAddr = (params.size() >= 2) ? params[1] : -1;
if (logicalAddr >= 0 && logicalAddr < memoryLimit) {
    int phys = translateLogicalToPhysical(logicalAddr, &mainMemory[pcbAddress]);
    mainMemory[phys] = value;
    registerValue    = value;
    // TODO: Remove this debug print
    cout << "stored\n";
    cout << "Logical address " << logicalAddr
         << " translated to physical address " << phys
         << " for Process " << processID << "\n";
} else {
    cout << "store error!\n";
}
++cpuCyclesUsed; ++globalClock; ++localCycles;
break;
}
case 4: {  // load
int logicalAddr = (params.size() >= 1) ? params[0] : -1;

// TODO: Remove this debug print
for(int i = 0; i < params.size(); i++)
{
    cout << params[i] << " ";
}
cout << logicalAddr << "\n";
if (logicalAddr >= 0) {
    int phys = translateLogicalToPhysical(logicalAddr, &mainMemory[pcbAddress]);
    registerValue = mainMemory[phys];
    
    // TODO: Remove this debug print
    cout << "loaded\n";
    cout << "Logical address " << logicalAddr
         << " translated to physical address " << phys
         << " for Process " << processID << "\n";
} 
else {
    cout << "load error!\n";
}
++cpuCyclesUsed; ++globalClock; ++localCycles;
break;
}
default:
cerr << "ERROR: Unknown opcode " << opcode << "\n";
}
++programCounter;
}

// ─── 3.  Time slice check ───────────────────────────────────────────
if (programCounter < numOpcodes) {        // still runnable
cout << "Process " << processID
 << " has a TimeOUT interrupt and is moved to the ReadyQueue.\n";
readyQueue.push(pcbAddress);
return;
}

// ─── 4.  Normal termination ────────────────────────────────────────
state          = TERMINATED;
programCounter = -1;
cout << "Process ID: " << processID << "\n";
    cout << "State: TERMINATED\n";
    cout << "Program Counter: "
         << (instrBaseRel - 1) << "\n";
    cout << "Instruction Base: " << instrBaseRel << "\n";
    cout << "Data Base: " << dataBaseRel  << "\n";
    cout << "Memory Limit: " << memoryLimit << "\n";
    cout << "CPU Cycles Used: " << cpuCyclesUsed << "\n";
    cout << "Register Value: " << registerValue << "\n";
    cout << "Max Memory Needed: "
         << mainMemory[ translateLogicalToPhysical(LOG_MAXMEM,
                                                    &mainMemory[pcbAddress]) ] << "\n";
    cout << "Main Memory Base: "
         << mainMemory[ translateLogicalToPhysical(LOG_MAINBASE,
                                                    &mainMemory[pcbAddress]) ] << "\n";

    int startTime = processStartTime[processID];
    int totalExec = globalClock - startTime;
    cout << "Total CPU Cycles Consumed: " << totalExec << "\n";

        // ── summary line required by project spec ───────────────────────────
        int endTime   = globalClock;                      // already includes final cycles
    
        cout << "Process " << processID
             << " terminated. Entered running state at: "
             << startTime << ". Terminated at: "
             << endTime   << ". Total Execution Time: "
             << totalExec << ".\n";
    

cout << "Process " << processID << " terminated and freed memory blocks.\n";

// Mark its blocks free
for (list<MemBlock>::iterator it = memoryBlocks.begin();
it != memoryBlocks.end(); ++it)
{
if (it->processID == processID) it->processID = -1;
}




// Coalesce adjacent free blocks (post‑termination)
for (list<MemBlock>::iterator it = memoryBlocks.begin();
it != memoryBlocks.end(); )
{
list<MemBlock>::iterator nxt = it; ++nxt;
if (nxt != memoryBlocks.end() && it->processID == -1 && nxt->processID == -1) {
it->size += nxt->size;
memoryBlocks.erase(nxt);
} else {
++it;
}
}

}





int main() {
    ios::sync_with_stdio(false);
    cin.tie(nullptr);

    int maxMemory, CPUAllocated, contextSwitchTime, numProcesses;
    cin >> maxMemory >> CPUAllocated >> contextSwitchTime >> numProcesses;

    queue<PCB> newJobQueue;
    for (int i = 0; i < numProcesses; ++i) {
        PCB proc;
        cin >> proc.processID >> proc.maxMemoryNeeded >> proc.instrStreamLength;
        string line; 
        getline(cin, line);
        if (line.empty()) {
            getline(cin, line);
        }
        stringstream ss(line);
        int token;
        while (ss >> token) {
            proc.tokens.push_back(token);
        }
        newJobQueue.push(proc);
    }

    vector<int> mainMemory(maxMemory, -1);
    queue<int> readyQueue;
    queue<waitingInfo> ioWaitingQueue;

    // start with one big free block
    list<MemBlock> memoryBlocks;
    memoryBlocks.push_back( MemBlock(-1, 0, maxMemory) );

    // initial load
    loadJobsToMemory(newJobQueue,
                     readyQueue,
                     mainMemory,
                     maxMemory,
                     memoryBlocks);

    // TODO: Remove this giant memory dump. It is causing the massive list of numbers in your output.
    // dump memory cells (for debug)
    for (int i = 0; i < maxMemory; ++i) {
        cout << i << " : " << mainMemory[i] << "\n";
    }

    int globalClock = contextSwitchTime;

    // scheduling loop
    while (!readyQueue.empty() || !ioWaitingQueue.empty()) {
        if (!readyQueue.empty()) {
            int pcbAddress = readyQueue.front();
            readyQueue.pop();
            executeCPU(pcbAddress,
                       mainMemory,
                       CPUAllocated,
                       contextSwitchTime,
                       globalClock,
                       readyQueue,
                       ioWaitingQueue,
                       memoryBlocks);

            // attempt to load more if someone freed memory
            if (mainMemory[pcbAddress + 4] == TERMINATED) {
                loadJobsToMemory(newJobQueue,
                                 readyQueue,
                                 mainMemory,
                                 maxMemory,
                                 memoryBlocks);
            }
        }

        // handle completed I/O
        int ioQueueSize = static_cast<int>(ioWaitingQueue.size());
        for (int j = 0; j < ioQueueSize; ++j) {
            waitingInfo info = ioWaitingQueue.front();
            ioWaitingQueue.pop();

            // segment‑table size is stored at logical 0
            int segSize        = mainMemory[ info.pcbAddress ];
            int logPIDIndex    = segSize + 1;              // N + 0 (processID)
            int physPID        = translateLogicalToPhysical(logPIDIndex, &mainMemory[info.pcbAddress]);
            int pid            = mainMemory[physPID];

            if (globalClock >= info.startTime + info.waitDuration &&
                mainMemory[translateLogicalToPhysical(segSize + 2, &mainMemory[info.pcbAddress]) ] != TERMINATED)
            {
                // TODO: Remove this debug print
                cout << "print\n";
                cout << "Process " << pid
                     << " completed I/O and is moved to the ReadyQueue.\n";
                readyQueue.push(info.pcbAddress);
            }
            else if (mainMemory[ translateLogicalToPhysical(segSize + 2, &mainMemory[info.pcbAddress]) ] != TERMINATED) 
            {
                ioWaitingQueue.push(info);
            }
        }

        globalClock += contextSwitchTime;
    }

    cout << "Total CPU time used: " << globalClock << ".\n";
    return 0;
}