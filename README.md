# Virtual memory simulator

A C++ simulation of paged virtual memory: a page table over a small physical memory, a disk file as backing store, and five page replacement algorithms. Four threads sort parts of one large array through the virtual memory, and the program reports page faults and disk traffic for each thread.

Written in spring 2020 as the final project for CSE 312 Operating Systems at Gebze Technical University.

## How it works

- `PageTable` (`page-table.h`) maps virtual page numbers to frames. Each entry has referenced, modified and present bits and a frame number.
- `VirtualMemory` (`virtual-memory.h`) stores ints in a physical array of frames and keeps the full virtual space in a disk file. `get` and `set` take a virtual index and the name of the calling thread; a missing page takes a free frame or triggers a replacement. Modified pages are written back to disk before eviction.
- Page replacement (`page-repl-algorithm.h`), one class per algorithm on a common base:
  - `NRU`: sorts present pages into four classes by referenced/modified bits and evicts a random page from the lowest class. Referenced bits are cleared every 10 memory accesses.
  - `FIFO`: evicts the page that was loaded first.
  - `SC` (second chance): FIFO, but a referenced page is moved to the back of the queue with its bit cleared.
  - `LRU`: keeps pages ordered by last access and evicts the least recently used one.
  - `WSClock`: walks a circular list, skips referenced pages (clearing the bit) and evicts a page whose last use is older than tau (500 ns).
- Allocation policy: `global` lets any thread evict any frame. `local` splits physical memory into equal fixed partitions, one per thread, and each thread replaces only its own frames.
- The workload (`paging-simulation.h`): fill the array with random numbers, then sort its four quarters in parallel with bubble sort, quick sort, merge sort and index sort (one `std::thread` each, sharing a mutex around memory access), then check that every quarter is sorted.
- At the end it prints reads, writes, page misses, page replacements, disk page reads and disk page writes for the fill step, each sort and the final check, followed by the elapsed time.

## Building

Needs `g++` with C++11. The Makefile builds with AddressSanitizer and `-Werror`.

```
cd source
make
```

This produces `sortArrays`.

## Running

```
./sortArrays <frame_size> <num_physical> <num_virtual> <algorithm> <policy> <print_int> <disk_file>
```

| Argument | Meaning |
| --- | --- |
| `frame_size` | log2 of the page size, in ints |
| `num_physical` | log2 of the number of physical frames (at least 4 frames) |
| `num_virtual` | log2 of the number of virtual pages |
| `algorithm` | `NRU`, `FIFO`, `SC`, `LRU` or `WSClock` |
| `policy` | `global` or `local` |
| `print_int` | how often to print the page table, in memory accesses; `0` prints the current thread's working-set size on every access instead (LRU only) |
| `disk_file` | file used as backing store; it is truncated at start |

`paging-simulation.h` also has three experiments that are commented out in `program.cpp`: `findOptimalSize` (the page size with the fewest replacements per sort, using FIFO), `findOptimalAlgorithm` (the algorithm with the lowest mean replacement count per sort) and `workingSetData` (working-set sizes under LRU with local allocation). Uncomment them in `program.cpp` to run them; the command-line arguments are still required but not used by them. The results are in `report.pdf`, section 3.

## Working-set graphs

`graph-script/graph_generator.py` reads a `test.txt` of `<sort name> <working-set size>` lines (what the program prints with `LRU`, `local` and `print_int` set to `0`, as `workingSetData` does) and plots the working-set size w(k, t) for the four sorts in a 2x2 grid. It uses pandas, matplotlib, scipy and numpy.

## Layout

- `source/program.cpp`: entry point
- `source/paging-simulation.h`: the workload, sorting threads and experiments
- `source/virtual-memory.h`, `source/page-table.h`, `source/page-repl-algorithm.h`: the memory model and algorithms
- `graph-script/graph_generator.py`: working-set plots
- `report.pdf`: design notes and results

## License

MIT, see LICENSE.
