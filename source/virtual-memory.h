#ifndef VIRTUAL_MEMORY_H
#define VURTUAL_MEMORY_H

#include <string>
#include <fstream>
#include <cmath>
#include <iostream>
#include <cstdlib>
#include <chrono>
#include "page-repl-algorithm.h"
#include "page-table.h"

typedef PageReplAlgorithm::Stats Stats;

class VirtualMemory
{
public:
    VirtualMemory(unsigned int, unsigned int, unsigned int, std::string, std::string, int, std::string);
    ~VirtualMemory();

    void set(unsigned int index, int value, char *tName);
    int get(unsigned int index, char *tName);
    void fill(char *tName);
    void setPartition(std::vector<char *>);
    void resetPartition();
    void printStats() const;
    const std::map<std::string, Stats> &getStats() const;

private:
    unsigned int frame_size_;
    unsigned int num_physical_;
    unsigned int num_virtual_;
    PageReplAlgorithm *algorithm_;
    bool policy_local_;
    int print_period_;
    int print_count_;
    std::fstream disc_;
    std::string disc_name_;

    unsigned int virtual_size_;
    unsigned int physical_size_;

    int *memory_; /* physical memory */
    PageTable *page_table_;
    unsigned int item_count_;

    void checkPowerOfTwo(unsigned int);
    void initMemory();
    void initPageTable();
    void initAlgorithm(std::string);
    void initAllocPolicy(std::string);
    void initDisc(std::string);

    void print();

    /* pre-defined string literals for parse command-line args */
    static const std::string kNRU;
    static const std::string kFIFO;
    static const std::string kSC;
    static const std::string kLRU;
    static const std::string kWSCLOCK;
    static const std::string kGLOBAL;
    static const std::string kLOCAL;
};

const std::string VirtualMemory::kNRU = "NRU";
const std::string VirtualMemory::kFIFO = "FIFO";
const std::string VirtualMemory::kSC = "SC";
const std::string VirtualMemory::kLRU = "LRU";
const std::string VirtualMemory::kWSCLOCK = "WSClock";
const std::string VirtualMemory::kGLOBAL = "global";
const std::string VirtualMemory::kLOCAL = "local";

VirtualMemory::VirtualMemory(unsigned int frameSize, unsigned int numPhysical, unsigned int numVirtual,
                             std::string pageReplacement, std::string policyName, int printPeriod,
                             std::string discName)
    : frame_size_(frameSize),
      num_physical_(numPhysical),
      num_virtual_(numVirtual),
      print_period_(printPeriod),
      print_count_(0)
{
    checkPowerOfTwo(frame_size_);
    checkPowerOfTwo(num_physical_);
    checkPowerOfTwo(num_virtual_);
    if (num_physical_ < 4)
        throw std::logic_error("num physical must be at least 4");

    initMemory();
    initPageTable();
    initAllocPolicy(policyName);
    initDisc(discName);
    initAlgorithm(pageReplacement);

    srand(1000);
}
void VirtualMemory::checkPowerOfTwo(unsigned int n)
{
    bool power_of_two = true;
    if (n == 0)
        throw std::logic_error("bad input: not power of 2");

    power_of_two = (std::ceil(std::log2(n)) == std::floor(std::log2(n)));

    if (!power_of_two)
        throw std::logic_error("bad input: not power of 2");
}

void VirtualMemory::initAlgorithm(std::string algorithmName)
{
    if (kNRU == algorithmName)
        algorithm_ = new NRU(page_table_, memory_, &disc_, policy_local_);
    else if (kFIFO == algorithmName)
        algorithm_ = new FIFO(page_table_, memory_, &disc_, policy_local_);
    else if (kSC == algorithmName)
        algorithm_ = new SC(page_table_, memory_, &disc_, policy_local_);
    else if (kLRU == algorithmName)
        algorithm_ = new LRU(page_table_, memory_, &disc_, policy_local_);
    else if (kWSCLOCK == algorithmName)
        algorithm_ = new WSClock(page_table_, memory_, &disc_, policy_local_);
    else
        throw std::logic_error("no such algorithm!");
}

void VirtualMemory::initMemory()
{
    virtual_size_ = frame_size_ * num_virtual_;
    physical_size_ = frame_size_ * num_physical_;
    memory_ = new int[physical_size_];
    item_count_ = 0;

    for (size_t i = 0; i < physical_size_; i++)
        memory_[i] = 0;
}

void VirtualMemory::initAllocPolicy(std::string policy)
{
    if (policy == kGLOBAL)
        policy_local_ = false;
    else if (policy == kLOCAL)
        policy_local_ = true;
    else
        throw std::logic_error("bad input for allocation policy!");
}

void VirtualMemory::initDisc(std::string discName)
{
    /* create the disc */
    disc_.open(discName, std::ios::out | std::ios::in | std::ios::binary | std::ios::trunc);
    disc_name_ = discName;

    int *disc_raw = new int[virtual_size_];

    /* reset */
    for (size_t i = 0; i < virtual_size_; i++)
        disc_raw[i] = 0;

    disc_.write((char *)disc_raw, virtual_size_ * sizeof(int));
    delete[] disc_raw;
}

VirtualMemory::~VirtualMemory()
{
    delete algorithm_;
    delete[] memory_;
    delete page_table_;
    disc_.close();
}

void VirtualMemory::initPageTable()
{
    page_table_ = new PageTable(frame_size_, num_physical_, num_virtual_);
}

int VirtualMemory::get(unsigned int index, char *tName)
{
    if (!page_table_->isPresent(index))
    {
        int physical_index = algorithm_->findIndex(tName);
        if (physical_index != -1)
        {
            page_table_->set(index, physical_index);
            algorithm_->recordNew(index);
            algorithm_->readFrame(index);
        }
        else /* page-table is full. replace */
            algorithm_->replace(index);
    }

    unsigned int address = page_table_->get(index);
    algorithm_->recordGet(index, tName);
    print(); // FIXME.
    return memory_[address];
}

void VirtualMemory::set(unsigned int index, int value, char *tName)
{
    if (!page_table_->isPresent(index))
    {
        int physical_index = algorithm_->findIndex(tName);
        if (physical_index != -1) /* empty slot is found */
        {
            page_table_->set(index, physical_index);
            algorithm_->recordNew(index);
        }
        else /* page-table is full. replace */
            algorithm_->replace(index);
    }

    unsigned int address = page_table_->get(index);
    assert(address < physical_size_);
    algorithm_->recordSet(index, tName);
    print();
    memory_[address] = value;
}

void VirtualMemory::fill(char *tName)
{
    for (size_t i = 0; i < virtual_size_; i++)
        set(i, rand(), tName);
}

void VirtualMemory::setPartition(std::vector<char *> tNames)
{

    unsigned int partition_size = num_physical_ / tNames.size();

    for (size_t i = 0; i < tNames.size(); ++i)
    {
        unsigned int lower_bound = i * partition_size;
        unsigned int upper_bound = (i + 1) * partition_size;
        algorithm_->addWorkingSet(tNames[i], lower_bound, upper_bound);
    }
}

void VirtualMemory::resetPartition()
{
    algorithm_->delWorkingSets();
}

void VirtualMemory::printStats() const
{
    algorithm_->printStats();
}

const std::map<std::string, Stats> &VirtualMemory::getStats() const
{
    return algorithm_->getStats();
}

void VirtualMemory::print()
{
    if (print_period_ == 0)
        algorithm_->workingSetSize();
    else if (print_count_++ == print_period_)
    {
        page_table_->print();
        print_count_ = 0;
    }
}

#endif