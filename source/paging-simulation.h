#ifndef PAGING_SIMULATION_H
#define PAGING_SIMULATION_H

#include <thread>
#include <mutex>
#include <chrono>
#include "virtual-memory.h"

#define THREAD_NUM 4

// TODO. all bonus parts will be done here.

class PagingSimulation
{
public:
    PagingSimulation();
    PagingSimulation(int, char const *[]);
    ~PagingSimulation();
    friend class BubbleSorter;

    void simulate();
    void findOptimalSize();
    void findOptimalAlgorithm();
    void workingSetData();

    enum class Quarter
    {
        FIRST,
        SECOND,
        THIRD,
        FOURTH,
    };

private:
    std::thread sorter_threads_[THREAD_NUM]; /* 4 different sorting threads */
    std::mutex *memory_mutex_;
    VirtualMemory *memory_;

    unsigned int memory_size_;

    void bubbleSort();
    void quickSort();
    void mergeSort();
    void indexSort();

    /* helper functions for sorting algorithms */
    void swap(unsigned int, unsigned int, char *tName);
    void getBounds(Quarter, unsigned int &, unsigned int &);

    /* quick sort helpers */
    int partition(int low, int high);
    void quickSortHelper(int low, int high);

    /* merge sort helpers */
    void merge(int low, int high, int mid);
    void mergeSortHelper(int l, int r);

    void initMemory(int argc, char const *argv[]);

    bool check();
    static const Quarter QUARTERS[THREAD_NUM];
    static const std::string QUARTER_NAMES[THREAD_NUM];
    static const std::string ALGORITHM_NAMES[5];

    void startSorters(std::vector<Stats> sorters[THREAD_NUM], unsigned int, std::string, std::string, int);

    void print();

    static char kFill[];
    static char kBubble[];
    static char kQuick[];
    static char kMerge[];
    static char kIndex[];
    static char kCheck[];
};

const PagingSimulation::Quarter PagingSimulation::QUARTERS[] = {
    PagingSimulation::Quarter::FIRST,
    PagingSimulation::Quarter::SECOND,
    PagingSimulation::Quarter::THIRD,
    PagingSimulation::Quarter::FOURTH,
};

const std::string PagingSimulation::QUARTER_NAMES[] = {"bubble", "quick", "merge", "index"};
const std::string PagingSimulation::ALGORITHM_NAMES[] = {"NRU", "FIFO", "SC", "LRU", "WSClock"};

char PagingSimulation::kFill[] = "fill";
char PagingSimulation::kBubble[] = "bubble";
char PagingSimulation::kQuick[] = "quick";
char PagingSimulation::kMerge[] = "merge";
char PagingSimulation::kIndex[] = "index";
char PagingSimulation::kCheck[] = "check";

PagingSimulation::PagingSimulation() : memory_mutex_(new std::mutex()), memory_(nullptr)
{
    /* */
}

PagingSimulation::PagingSimulation(int argc, char const *argv[]) : memory_mutex_(new std::mutex())
{
    initMemory(argc, argv);
}

void PagingSimulation::getBounds(PagingSimulation::Quarter quarter,
                                 unsigned int &lower_bound, unsigned int &upper_bound)
{
    unsigned int quarter_size = memory_size_ / 4;
    switch (quarter)
    {
    case Quarter::FIRST:
        lower_bound = 0;
        upper_bound = quarter_size;
        break;
    case Quarter::SECOND:
        lower_bound = quarter_size;
        upper_bound = quarter_size * 2;
        break;
    case Quarter::THIRD:
        lower_bound = quarter_size * 2;
        upper_bound = quarter_size * 3;
        break;
    case Quarter::FOURTH:
        lower_bound = quarter_size * 3;
        upper_bound = quarter_size * 4;
        break;

    default:
        break;
    }
}

void PagingSimulation::bubbleSort()
{
    unsigned int lower_bound, upper_bound;
    getBounds(Quarter::FIRST, lower_bound, upper_bound);

    for (size_t i = lower_bound; i < upper_bound - 1; i++)
    {
        // std::cout << "\t" << i << "/" << upper_bound << std::endl;
        for (size_t j = lower_bound; j < upper_bound - i - 1; j++)
        {
            memory_mutex_->lock();
            if (memory_->get(j, kBubble) > memory_->get(j + 1, kBubble))
                swap(j, j + 1, kBubble);
            memory_mutex_->unlock();
        }
    }

    std::cout << "Bubble Sort finished!" << std::endl;
}

void PagingSimulation::swap(unsigned int i1, unsigned int i2, char *tName)
{
    int val1 = memory_->get(i1, tName);
    int val2 = memory_->get(i2, tName);
    memory_->set(i1, val2, tName);
    memory_->set(i2, val1, tName);
}

int PagingSimulation::partition(int low, int high)
{
    int pivot = memory_->get(high, kQuick);
    int i = low - 1;

    for (int j = low; j <= high - 1; j++)
    {
        if (memory_->get(j, kQuick) < pivot)
        {
            i++;
            swap(i, j, kQuick);
        }
    }
    swap(i + 1, high, kQuick);
    return i + 1;
}

void PagingSimulation::simulate()
{
    std::cout << "Starting ..\n";
    std::chrono::steady_clock sc;
    auto start = sc.now();

    /* random filling */
    std::cout << "Filling the array...\n";
    memory_->setPartition({kFill});
    memory_->fill(kFill);
    memory_->resetPartition();

    /* sorting quarters */
    memory_->setPartition({kBubble, kQuick, kMerge, kIndex});

    std::cout << "Sorting...\n";
    sorter_threads_[0] = std::thread(&PagingSimulation::bubbleSort, this);
    sorter_threads_[1] = std::thread(&PagingSimulation::quickSort, this);
    sorter_threads_[2] = std::thread(&PagingSimulation::mergeSort, this);
    sorter_threads_[3] = std::thread(&PagingSimulation::indexSort, this);

    /* wait for all quarters to finish */
    for (size_t i = 0; i < THREAD_NUM; i++)
        sorter_threads_[i].join();
    memory_->resetPartition();

    /* scan the array */
    std::cout << "Scanning the array...\n";
    memory_->setPartition({kCheck});
    if (check())
        std::cout << "Array is sorted!\n";
    else
        throw std::logic_error("Array is not sorted!\n");

    /* print page table stats */
    // TODO. get stats and compare stuff.
    memory_->printStats();

    std::cout << "Simulation finished!\nElapsed time:\t";
    auto end = sc.now();
    auto time_span = static_cast<std::chrono::duration<double>>(end - start);
    std::cout << time_span.count() << " secs" << std::endl;
}

void PagingSimulation::quickSort()
{
    unsigned int lower_bound, upper_bound;
    getBounds(Quarter::SECOND, lower_bound, upper_bound);

    quickSortHelper(lower_bound, upper_bound - 1);
    std::cout << "Quick Sort finished!" << std::endl;
}

void PagingSimulation::quickSortHelper(int low, int high)
{
    if (low < high)
    {
        memory_mutex_->lock();
        int pi = partition(low, high);
        memory_mutex_->unlock();

        // std::cout << "\t " << low << "-" << high << std::endl;

        quickSortHelper(low, pi - 1);
        quickSortHelper(pi + 1, high);
    }
}

void PagingSimulation::merge(int l, int m, int r)
{
    int i, j, k;
    int n1 = m - l + 1;
    int n2 = r - m;

    int *L = new int[n1];
    int *R = new int[n2];

    for (i = 0; i < n1; i++)
        L[i] = memory_->get(l + i, kMerge);
    for (j = 0; j < n2; j++)
        R[j] = memory_->get(m + 1 + j, kMerge);

    i = 0;
    j = 0;
    k = l;
    while (i < n1 && j < n2)
    {
        if (L[i] <= R[j])
            memory_->set(k++, L[i++], kMerge);
        else
            memory_->set(k++, R[j++], kMerge);
    }

    while (i < n1)
        memory_->set(k++, L[i++], kMerge);

    while (j < n2)
        memory_->set(k++, R[j++], kMerge);

    delete[] L;
    delete[] R;
}

void PagingSimulation::mergeSortHelper(int l, int r)
{
    if (l < r)
    {
        int m = l + (r - l) / 2;

        mergeSortHelper(l, m);
        mergeSortHelper(m + 1, r);

        memory_mutex_->lock();
        merge(l, m, r);
        memory_mutex_->unlock();
    }
}

void PagingSimulation::mergeSort()
{
    unsigned int lower_bound, upper_bound;
    getBounds(Quarter::THIRD, lower_bound, upper_bound);

    mergeSortHelper(lower_bound, upper_bound - 1);
    std::cout << "Merge Sort finished!" << std::endl;
}

void PagingSimulation::initMemory(int argc, char const *argv[])
{
    /* parsing arguments and initializing the memory */
    if (argc < 8)
        throw std::logic_error("missing arguments!");

    try
    {
        unsigned int frame_size = std::stoi(argv[1]);
        unsigned int num_physical = std::stoi(argv[2]);
        unsigned int num_virtual = std::stoi(argv[3]);
        std::string page_replacement = argv[4];
        std::string alloc_policy = argv[5];
        unsigned int print_int = std::stoi(argv[6]);
        std::string disc_name = argv[7];

        frame_size = std::pow(2, frame_size);
        num_physical = std::pow(2, num_physical);
        num_virtual = std::pow(2, num_virtual);

        memory_size_ = num_virtual * frame_size;

        memory_ = new VirtualMemory(frame_size, num_physical, num_virtual,
                                    page_replacement, alloc_policy, print_int, disc_name);
    }
    catch (const std::exception &e)
    {
        throw std::logic_error(std::string("invalid argument!\t") + std::string(e.what()));
    }
}

PagingSimulation::~PagingSimulation()
{
    if (memory_ != nullptr)
        delete memory_;
    delete memory_mutex_;
}

void PagingSimulation::print()
{
    for (size_t i = 0; i < memory_size_; i++)
    {
        if (i % (memory_size_ / 4) == 0)
            std::cout << "\tquarter:" << std::endl;
        std::cout << memory_->get(i, kCheck) << std::endl;
    }
}

bool PagingSimulation::check()
{
    bool sorted = true;

    for (const auto &q : QUARTERS)
    {
        unsigned int lower_bound, upper_bound;
        getBounds(q, lower_bound, upper_bound);
        for (size_t i = lower_bound; i < upper_bound - 1 && sorted; i++)
            if (memory_->get(i, kCheck) > memory_->get(i + 1, kCheck))
                sorted = false;

        if (!sorted)
            break;
    }

    return sorted;
}

void PagingSimulation::indexSort()
{
    unsigned lower_bound, upper_bound;
    getBounds(Quarter::FOURTH, lower_bound, upper_bound);

    unsigned int partition_size = upper_bound - lower_bound;
    unsigned int *index = new unsigned int[partition_size];
    for (size_t i = 0; i < partition_size; i++)
        index[i] = i + lower_bound;

    /* find the sorted of indexes in the index array. */
    for (size_t i = 0; i < partition_size - 1; i++)
    {
        for (size_t j = i + 1; j < partition_size; j++)
        {
            memory_mutex_->lock();
            bool swap_need = memory_->get(index[i], kIndex) > memory_->get(index[j], kIndex);
            memory_mutex_->unlock();
            if (swap_need)
            {
                unsigned int temp = index[i];
                index[i] = index[j];
                index[j] = temp;
            }
        }
    }

    int *sorted_array = new int[partition_size];
    for (size_t i = 0; i < partition_size; i++)
    {
        memory_mutex_->lock();
        sorted_array[i] = memory_->get(index[i], kIndex);
        memory_mutex_->unlock();
    }

    delete[] index;

    /* arrange the original order according to the order in the index array. */
    for (size_t i = 0; i < partition_size; i++)
    {
        memory_mutex_->lock();
        memory_->set(i + lower_bound, sorted_array[i], kIndex);
        // std::cout << "print" << std::endl;
        memory_mutex_->unlock();
    }
    delete[] sorted_array;

    std::cout << "Index Sort finished!" << std::endl;
}

void PagingSimulation::startSorters(std::vector<Stats> sorters[THREAD_NUM], unsigned int times, std::string algorithm, std::string policy = "global", int print_period = -1)
{
    for (unsigned int i = 0; i < times - 1; i++)
    {
        unsigned int frame_size = std::pow(2, i);
        unsigned int physical_num = std::pow(2, (times + 1) - i);
        unsigned int virtual_num = std::pow(2, (times + 4) - i);
        std::cout << "Calculating stats... (frame size: " << frame_size;
        std::cout << ", physical memory: " << physical_num * frame_size;
        std::cout << ", virtual memory: " << virtual_num * frame_size << ")\n";

        memory_ = new VirtualMemory(frame_size, physical_num, virtual_num, algorithm, policy, print_period, "disc.dat");
        memory_size_ = virtual_num * frame_size;

        std::cout << "Filling...\n";
        memory_->setPartition({kFill});
        memory_->fill(kFill);
        memory_->resetPartition();

        memory_->setPartition({kBubble, kQuick, kMerge, kIndex});
        std::cout << "Sorting...\n";
        sorter_threads_[0] = std::thread(&PagingSimulation::bubbleSort, this);
        sorter_threads_[1] = std::thread(&PagingSimulation::quickSort, this);
        sorter_threads_[2] = std::thread(&PagingSimulation::mergeSort, this);
        sorter_threads_[3] = std::thread(&PagingSimulation::indexSort, this);

        /* wait for all quarters to finish */
        for (size_t i = 0; i < THREAD_NUM; i++)
            sorter_threads_[i].join();
        auto &stats = memory_->getStats();
        memory_->resetPartition();

        sorters[0].push_back(stats.at(kBubble));
        sorters[1].push_back(stats.at(kQuick));
        sorters[2].push_back(stats.at(kMerge));
        sorters[3].push_back(stats.at(kIndex));
        delete memory_;
    }
    memory_ = nullptr;
}

void PagingSimulation::findOptimalSize()
{
    if (memory_ != nullptr)
        delete memory_;
    std::vector<Stats> sorters[THREAD_NUM];

    startSorters(sorters, 13, "FIFO");

    for (size_t i = 0; i < THREAD_NUM; i++)
    {
        auto &sorter = sorters[i];
        auto it = std::min_element(sorter.begin(), sorter.end(),
                                   [](const Stats &s1, const Stats &s2) { return s1.page_repl < s2.page_repl; });
        unsigned int optimal_page_size = std::pow(2, std::distance(sorter.begin(), it));
        std::cout << "Optimal page size for " << QUARTER_NAMES[i] << " is: " << optimal_page_size << std::endl;
    }
}

void PagingSimulation::findOptimalAlgorithm()
{
    if (memory_ != nullptr)
        delete memory_;

    std::vector<unsigned int> means[THREAD_NUM];
    for (auto &algorithm : ALGORITHM_NAMES)
    {

        std::vector<Stats> sorters[THREAD_NUM];

        startSorters(sorters, 8, algorithm);

        for (size_t i = 0; i < THREAD_NUM; i++)
        {
            auto &sorter = sorters[i];
            unsigned int mean = 0;

            std::for_each(sorter.begin(), sorter.end(), [&mean](const Stats &s) { mean += s.page_repl; });
            means[i].push_back(mean / sorter.size());
        }
    }

    for (size_t i = 0; i < THREAD_NUM; i++)
    {
        auto &mean = means[i];
        auto it = std::min_element(mean.begin(), mean.end());
        std::string optimal_algorithm = ALGORITHM_NAMES[std::distance(mean.begin(), it)];
        std::cout << "Optimal algorithm " << QUARTER_NAMES[i] << " is: " << optimal_algorithm << std::endl;
    }
}

void PagingSimulation::workingSetData()
{
    if (memory_ != nullptr)
        delete memory_;

    unsigned int frame_size = 4;
    unsigned int physical_num = std::pow(2, 8);
    unsigned int virtual_num = std::pow(2, 10);

    memory_ = new VirtualMemory(frame_size, physical_num, virtual_num, "LRU", "local", 0, "disc.dat");
    memory_size_ = virtual_num * frame_size;

    memory_->setPartition({kFill});
    memory_->fill(kFill);
    memory_->resetPartition();

    memory_->setPartition({kBubble, kQuick, kMerge, kIndex});
    sorter_threads_[0] = std::thread(&PagingSimulation::bubbleSort, this);
    sorter_threads_[1] = std::thread(&PagingSimulation::quickSort, this);
    sorter_threads_[2] = std::thread(&PagingSimulation::mergeSort, this);
    sorter_threads_[3] = std::thread(&PagingSimulation::indexSort, this);

    /* wait for all quarters to finish */
    for (size_t i = 0; i < THREAD_NUM; i++)
        sorter_threads_[i].join();
    memory_->resetPartition();

        delete memory_;
    memory_ = nullptr;
}

#endif
