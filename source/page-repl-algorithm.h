/**
 * represents the page replacement algorithm interface.
 * @see virtual-memory.cpp
 ***/

#ifndef PAGE_REPL_ALGORITHM_H
#define PAGE_REPL_ALGORITHM_H

#include "page-table.h"
#include <cmath>
#include <fstream>
#include <vector>
#include <queue>
#include <map>
#include <set>
#include <list>
#include <algorithm>
#include <chrono>
#include <iostream>

class PageReplAlgorithm
{
public:
    PageReplAlgorithm(PageTable *pageTable, int *memory, std::fstream *disc, bool allocPolicy);
    virtual ~PageReplAlgorithm();

    void replace(unsigned int);
    virtual void recordGet(unsigned int, std::string);
    virtual void recordSet(unsigned int, std::string);
    virtual void recordNew(unsigned int);

    /* will only be implement for LRU to satisfy to bonus part */
    virtual void workingSetSize() const {};

    struct Stats /* keeps the count for each field */
    {
        unsigned int read;
        unsigned int write;
        unsigned int page_miss;
        unsigned int page_repl;
        unsigned int disc_read;
        unsigned int disc_write;
    };
    void printStats() const;
    const std::map<std::string, Stats> &getStats() const;

    struct LocalReplacementInfo
    {
    public:
        unsigned int lower_bound_, upper_bound_, local_free_index_;
    };

    virtual void addWorkingSet(std::string, unsigned int, unsigned int);
    virtual void delWorkingSets();
    int findIndex(std::string);
    void writeFrame(unsigned int, unsigned int);
    void readFrame(unsigned int);

protected:
    virtual unsigned int find() = 0;
    PageTable *page_table_;
    int *memory_;
    std::fstream *disc_;
    bool local_;
    std::string current_thread_;
    std::string current_stat_;
    unsigned int global_free_index_;

    /* pages are stored for local page replacement */
    std::map<std::string, LocalReplacementInfo> *threads_working_set_;
    std::map<std::string, Stats> stats_;

    bool inWorkingSet(unsigned int) const;
};

class NRU : public PageReplAlgorithm
{
public:
    NRU(PageTable *pageTable, int *memory, std::fstream *disc, bool allocPolicy);

    void recordGet(unsigned int, std::string);
    void recordSet(unsigned int, std::string);

private:
    unsigned int find();
    void handleTimer();

    unsigned int timer_;
    static const unsigned int kClockPeriod;
};

class FIFO : public PageReplAlgorithm
{
public:
    FIFO(PageTable *pageTable, int *memory, std::fstream *disc, bool allocPolicy);

    void recordNew(unsigned int);
    void delWorkingSets();

protected:
    virtual unsigned int find();
    std::map<std::string, std::queue<unsigned int>> queues_;
};

class SC : public FIFO
{
public:
    SC(PageTable *pageTable, int *memory, std::fstream *disc, bool allocPolicy);

private:
    unsigned int find();
};

class LRU : public PageReplAlgorithm
{
public:
    LRU(PageTable *pageTable, int *memory, std::fstream *disc, bool allocPolicy);

    void recordGet(unsigned int, std::string);
    void recordSet(unsigned int, std::string);
    void delWorkingSets();
    void updateLists(unsigned int);

    void workingSetSize() const;

private:
    unsigned int find();
    std::map<std::string, std::vector<unsigned int>> lists_;
};

class WSClock : public PageReplAlgorithm
{
public:
    WSClock(PageTable *pageTable, int *memory, std::fstream *disc, bool allocPolicy);

    void recordGet(unsigned int, std::string);
    void recordSet(unsigned int, std::string);
    void delWorkingSets();
    void updateLists(unsigned int);

    struct WSClockEntry
    {
    public:
        bool operator==(unsigned int _index) { return index == _index; };
        unsigned int index;
        std::chrono::steady_clock::time_point last_use;
    };

private:
    unsigned int find();

    /* to simulate circular linked list implementation */
    std::map<std::string, std::vector<WSClockEntry>> lists_;

    static const double kTau;
    static std::chrono::steady_clock clock_;
    bool isIdeal(WSClockEntry &);
};

PageReplAlgorithm::PageReplAlgorithm(PageTable *pageTable, int *memory, std::fstream *disc, bool allocPolicy)
    : page_table_(pageTable),
      memory_(memory),
      disc_(disc),
      local_(allocPolicy),
      global_free_index_(0)
{
    if (local_)
        threads_working_set_ = new std::map<std::string, LocalReplacementInfo>();

    srand(1000);
}

void PageReplAlgorithm::recordGet(unsigned int index, std::string tName)
{
    current_stat_ = current_thread_ = tName;
    auto &entry = page_table_->getEntry(index);
    entry.setReferenced(true);
    stats_[current_stat_].read++;
}

PageReplAlgorithm::~PageReplAlgorithm()
{
    if (local_)
        delete threads_working_set_;
}

void PageReplAlgorithm::recordSet(unsigned int index, std::string tName)
{
    current_stat_ = current_thread_ = tName;
    auto &entry = page_table_->getEntry(index);
    entry.setModified(true);
    entry.setReferenced(true);
    stats_[current_stat_].write++;
}

void PageReplAlgorithm::writeFrame(unsigned int virtual_high_order_bits, unsigned int physical_high_order_bits)
{
    unsigned int frame_size = page_table_->frame_size_;
    int *data = new int[frame_size];
    for (size_t i = 0; i < frame_size; i++)
        data[i] = memory_[physical_high_order_bits | i];

    /* write one page */
    disc_->seekp(std::ios::beg + (virtual_high_order_bits * sizeof(int)));
    // assert(disc_->tellp() == virtual_high_order_bits * sizeof(int));
    assert(virtual_high_order_bits % frame_size == 0);
    disc_->write((char *)data, frame_size * sizeof(int));
    delete[] data;
}

void PageReplAlgorithm::readFrame(unsigned int address)
{
    unsigned int index = page_table_->getHighOrder(address);
    assert(index < page_table_->num_virtual_);
    auto &entry = page_table_->table_[index];
    unsigned int virtual_high_order_bits = index << page_table_->low_order_size_;
    unsigned int physical_high_order_bits = entry.getFrameNumber() << page_table_->low_order_size_;

    disc_->seekg(std::ios::beg + (virtual_high_order_bits * sizeof(int)));
    // assert(disc_->tellg() == virtual_high_order_bits);
    disc_->read((char *)(memory_ + physical_high_order_bits), sizeof(int) * page_table_->frame_size_);
    stats_[current_stat_].disc_read++;
}

void PageReplAlgorithm::replace(unsigned int index)
{

    stats_[current_stat_].page_repl++;
    unsigned int replace_idx = find();
    unsigned int frame_size = page_table_->frame_size_;

    auto &entry = page_table_->table_[replace_idx];
    assert(entry.isPresent());
    entry.setPresent(false); /* replaced entry is no longer present in the table */
    unsigned int virtual_high_order_bits = replace_idx << page_table_->low_order_size_;
    unsigned int physical_high_order_bits = entry.getFrameNumber() << page_table_->low_order_size_;

    if (entry.isModified()) /* write to disc if modified */
    {
        writeFrame(virtual_high_order_bits, physical_high_order_bits);
        entry.setModified(false);
        stats_[current_stat_].disc_write++;
    }

    /* read one page */
    page_table_->set(index, entry.getFrameNumber());
    virtual_high_order_bits = page_table_->getHighOrder(index) << page_table_->low_order_size_;
    assert(physical_high_order_bits == page_table_->getEntry(index).getFrameNumber() << page_table_->low_order_size_);
    disc_->seekg(std::ios::beg + (virtual_high_order_bits * sizeof(int)));
    disc_->read((char *)(memory_ + physical_high_order_bits), sizeof(int) * frame_size);
    recordNew(index);
    stats_[current_stat_].disc_read++;
}

void PageReplAlgorithm::addWorkingSet(std::string thread_name, unsigned int lower_bound, unsigned int upper_bound)
{

    if (local_)
    {
        /* first time */
        struct LocalReplacementInfo working_set
        {
            lower_bound, upper_bound, 0
        };
        threads_working_set_->insert({thread_name, working_set});
    }
    Stats stats{
        0, 0, 0, 0, 0, 0};
    stats_.insert({thread_name, stats});
}

void PageReplAlgorithm::delWorkingSets()
{
    if (local_)
        threads_working_set_->clear();
    else
        global_free_index_ = 0;

    for (size_t i = 0; i < page_table_->num_virtual_; i++)
    {
        auto &entry = page_table_->table_[i];
        if (entry.isPresent())
            writeFrame(i << page_table_->low_order_size_, entry.getFrameNumber() << page_table_->low_order_size_);
        entry.setPresent(false);
    }
}

void PageReplAlgorithm::printStats() const
{
    for (auto it = stats_.begin(); it != stats_.end(); it++)
    {
        std::cout << "{ Statistics for " + it->first + " }\n";
        std::cout << "\t* Number of reads " << it->second.read << "\n";
        std::cout << "\t* Number of writes " << it->second.write << "\n";
        std::cout << "\t* Number of page misses " << it->second.page_miss << "\n";
        std::cout << "\t* Number of page replacements " << it->second.page_repl << "\n";
        std::cout << "\t* Number of disk page reads " << it->second.disc_read << "\n";
        std::cout << "\t* Number of disk page writes " << it->second.disc_write << "\n"
                  << std::endl;
    }
}

const std::map<std::string, PageReplAlgorithm::Stats> &PageReplAlgorithm::getStats() const
{
    return stats_;
}

bool PageReplAlgorithm::inWorkingSet(unsigned int physical_index) const
{
    assert(local_);

    auto &working_set = threads_working_set_->at(current_thread_);
    return working_set.lower_bound_ <= physical_index && physical_index < working_set.upper_bound_;
}

int PageReplAlgorithm::findIndex(std::string tName)
{
    unsigned int lower_bound, upper_bound;
    unsigned int *free_index = nullptr;
    current_stat_ = current_thread_ = tName;
    if (local_)
    {
        auto &working_set = threads_working_set_->at(tName);
        free_index = &working_set.local_free_index_;
        lower_bound = working_set.lower_bound_;
        upper_bound = working_set.upper_bound_;
    }
    else
    {
        free_index = &global_free_index_;
        lower_bound = 0;
        upper_bound = page_table_->num_physical_;
    }

    stats_[current_stat_].page_miss++;
    unsigned int index = lower_bound + *free_index;

    if (index == upper_bound)
        return -1;
    else
    {
        (*free_index)++;
        return index;
    }
}

void PageReplAlgorithm::recordNew(unsigned int index)
{
    /* intentionally left blank for making this record optional. */
    index = index; /* dummy assignment to suppress warnings */
}

/* NRU implementation */

const unsigned int NRU::kClockPeriod = 10;

NRU::NRU(PageTable *pageTable, int *memory, std::fstream *disc, bool allocPolicy)
    : PageReplAlgorithm(pageTable, memory, disc, allocPolicy), timer_(0)
{
    /* intentionally left blank */
}

unsigned int NRU::find()
{
    /* 4 different classes: cartesian product of (referenced, modified) */
    std::vector<unsigned int> page_classes[4];

    for (size_t i = 0; i < page_table_->num_virtual_; i++)
    {
        auto &entry = page_table_->table_[i];

        bool cont = local_ ? inWorkingSet(entry.getFrameNumber()) : true; /* local page replacement policies applied if local */
        if (cont && entry.isPresent())
        {
            if (entry.isReferenced())
            {
                if (entry.isModified())
                    page_classes[3].push_back(i); /* class 3: referenced & modified */
                else
                    page_classes[2].push_back(i); /* class 2: referenced & not modified */
            }
            else
            {
                if (entry.isModified())
                    page_classes[1].push_back(i); /* class 1: not referenced & modified */
                else
                    page_classes[0].push_back(i); /* class 0: not referenced & not modified */
            }
        }
    }

    bool found = false;
    unsigned int replace_idx = 0;
    for (auto &c : page_classes)
    {
        if (!c.empty())
        {
            found = true;
            /* pick a random item from the given class */
            replace_idx = c[rand() % c.size()];
            break;
        }
    }

    assert(found);

    return replace_idx;
}

void NRU::handleTimer()
{
    timer_++;

    /* refresh reference bits in every clock period */
    if (timer_ == kClockPeriod)
    {
        for (size_t i = 0; i < page_table_->num_virtual_; i++)
            page_table_->table_[i].setReferenced(false);
        timer_ = 0;
    }
}

void NRU::recordGet(unsigned int index, std::string tName)
{
    PageReplAlgorithm::recordGet(index, tName);
    handleTimer();
}

void NRU::recordSet(unsigned int index, std::string tName)
{
    PageReplAlgorithm::recordSet(index, tName);
    handleTimer();
}

/* FIFO implementation */
FIFO::FIFO(PageTable *pageTable, int *memory, std::fstream *disc, bool allocPolicy)
    : PageReplAlgorithm(pageTable, memory, disc, allocPolicy)
{
    /* intentionally left blank */
}

void FIFO::recordNew(unsigned int index)
{
    if (!local_)
        current_thread_ = "global";

    if (queues_.find(current_thread_) == queues_.end()) /* first time */
        queues_.insert({current_thread_, std::queue<unsigned int>()});
    assert(queues_[current_thread_].front() != page_table_->getHighOrder(index));
    queues_[current_thread_].push(page_table_->getHighOrder(index));
}

unsigned int FIFO::find()
{
    if (!local_)
        current_thread_ = "global";
    auto &queue = queues_[current_thread_];
    assert(!queue.empty());
    unsigned int index = queue.front();
    queue.pop();
    return index;
}

void FIFO::delWorkingSets()
{
    PageReplAlgorithm::delWorkingSets();
    queues_.clear();
}

/* SC implementation */
SC::SC(PageTable *pageTable, int *memory, std::fstream *disc, bool allocPolicy)
    : FIFO(pageTable, memory, disc, allocPolicy)
{
    /* intentionally left blank */
}

unsigned int SC::find()
{
    unsigned int index = FIFO::find();
    auto &entry = page_table_->table_[index];
    assert(entry.isPresent());
    if (entry.isReferenced())
    {
        entry.setReferenced(false);
        queues_[current_thread_].push(index); /* give a second change by putting back into line */
        return SC::find();
    }
    else
        return index;
}

/* LRU implementation */

LRU::LRU(PageTable *pageTable, int *memory, std::fstream *disc, bool allocPolicy)
    : PageReplAlgorithm(pageTable, memory, disc, allocPolicy)
{
    /* intentionally left blank */
}

void LRU::recordGet(unsigned int index, std::string tName)
{
    PageReplAlgorithm::recordGet(index, tName);
    updateLists(index);
}

void LRU::recordSet(unsigned int index, std::string tName)
{
    PageReplAlgorithm::recordSet(index, tName);
    updateLists(index);
}

void LRU::updateLists(unsigned int index)
{
    if (!local_)
        current_thread_ = "global";

    if (lists_.find(current_thread_) == lists_.end()) /* first time */
        lists_.insert({current_thread_, std::vector<unsigned int>()});

    // unsigned int real_index = index;
    index = page_table_->getHighOrder(index);
    auto &list = lists_[current_thread_];
    auto it = std::find(list.begin(), list.end(), index);

    if (it != list.end()) /* item exists */
        list.erase(it);
    list.push_back(index);
}

unsigned int LRU::find()
{
    if (!local_)
        current_thread_ = "global";

    auto &list = lists_[current_thread_];

    unsigned int index = list.front();
    list.erase(list.begin());
    return index;
}

void LRU::delWorkingSets()
{
    PageReplAlgorithm::delWorkingSets();
    lists_.clear();
}

void LRU::workingSetSize() const
{
    if (current_thread_ != "fill" && current_thread_ != "check")
    {
        unsigned int ws_size = lists_.at(current_thread_).size();
            std::cout << current_thread_ << " " << ws_size << std::endl;
    }
}

/* WSClock implementation */

WSClock::WSClock(PageTable *pageTable, int *memory, std::fstream *disc, bool allocPolicy)
    : PageReplAlgorithm(pageTable, memory, disc, allocPolicy)
{
    /* intentionally left blank */
}

const double WSClock::kTau = 500;
std::chrono::steady_clock WSClock::clock_;

void WSClock::recordGet(unsigned int index, std::string tName)
{
    PageReplAlgorithm::recordGet(index, tName);
    updateLists(index);
}

void WSClock::recordSet(unsigned int index, std::string tName)
{
    PageReplAlgorithm::recordSet(index, tName);
    updateLists(index);
}

void WSClock::updateLists(unsigned int index)
{
    if (!local_)
        current_thread_ = "global";

    if (lists_.find(current_thread_) == lists_.end()) /* first time */
        lists_.insert({current_thread_, std::vector<WSClockEntry>()});

    index = page_table_->getHighOrder(index);
    auto &list = lists_[current_thread_];
    auto it = std::find(list.begin(), list.end(), index);

    if (it != list.end())
        it->last_use = clock_.now();
    else
        list.push_back({index, clock_.now()});
}

unsigned int WSClock::find()
{
    if (!local_)
        current_thread_ = "global";

    auto &list = lists_[current_thread_];

    auto &entry = list.front();
    list.erase(list.begin());

    if (isIdeal(entry))
        return entry.index;
    else
    {
        /* repaat the process with next page */
        list.push_back(entry);
        return WSClock::find();
    }
}

void WSClock::delWorkingSets()
{
    PageReplAlgorithm::delWorkingSets();
    lists_.clear();
}

bool WSClock::isIdeal(WSClockEntry &clock_entry)
{
    auto &table_entry = page_table_->table_[clock_entry.index];
    if (table_entry.isReferenced())
    {
        table_entry.setReferenced(false);
        return false;
    }
    else
    {
        /* comparing with tau */
        auto time_span = std::chrono::duration_cast<std::chrono::nanoseconds>(clock_.now() - clock_entry.last_use).count();
        // std::cout << "time_span: " << time_span << "  kTau" << kTau << std::endl;
        return time_span > kTau;
    }
}

#endif