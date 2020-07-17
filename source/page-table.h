#ifndef PAGE_TABLE_H
#define PAGE_TABLE_H

#include <cmath>
#include <cassert>
#include <vector>
#include "page-repl-algorithm.h"

class PageTable
{
public:
    friend class PageReplAlgorithm;
    friend class NRU;
    friend class FIFO;
    friend class SC;
    friend class LRU;
    friend class WSClock;

    PageTable(unsigned int, unsigned int, unsigned int);
    ~PageTable();

    /* address oriented in page-table */
    bool isPresent(unsigned int) const;
    void setModified(unsigned int address);

    void print() const;

    unsigned int get(unsigned int) const;
    void set(unsigned int, unsigned int);

    class Entry
    {
        friend class PageTable;

    public:
        Entry();

        void setReferenced(bool);
        void setModified(bool);
        void setPresent(bool);

        bool isReferenced() const;
        bool isModified() const;
        bool isPresent() const;

        unsigned int getFrameNumber() const;

    private:
        bool referenced_;
        bool modified_;
        bool present_;
        unsigned int page_frame_number_;
    };

private:
    unsigned int frame_size_;
    unsigned int num_virtual_;
    unsigned int num_physical_;
    unsigned int high_order_size_;
    unsigned int low_order_size_;

    unsigned int high_order_mask_;
    unsigned int low_order_mask_;

    unsigned int virtual_address_bits_;
    unsigned int physical_address_bits_;

    Entry *table_;

    void initHighOrderMask();
    void initLowOrderMask();
    unsigned int getHighOrder(unsigned int) const;
    unsigned int getLowOrder(unsigned int) const;

    void setNthBit(unsigned int &, unsigned int);

    Entry &getEntry(unsigned int);
    const Entry &getEntry(unsigned int) const;
};

PageTable::PageTable(unsigned int frameSize, unsigned int numPhysical, unsigned int numVirtual)
    : frame_size_(frameSize),
      num_virtual_(numVirtual),
      num_physical_(numPhysical)
{
    physical_address_bits_ = std::log2(numPhysical * frame_size_);
    virtual_address_bits_ = std::log2(numVirtual * frame_size_);

    low_order_size_ = std::log2(frame_size_);
    high_order_size_ = virtual_address_bits_ - high_order_size_;

    initHighOrderMask();
    initLowOrderMask();

    table_ = new Entry[num_virtual_];
}

bool PageTable::isPresent(unsigned int address) const
{
    return getEntry(address).present_;
}

void PageTable::setModified(unsigned int address)
{
    getEntry(address).modified_ = true;
}

unsigned int PageTable::get(unsigned int address) const
{
    assert(getEntry(address).present_);
    unsigned int high_order_bits = getEntry(address).page_frame_number_ << low_order_size_;
    unsigned int low_order_bits = getLowOrder(address);
    return high_order_bits | low_order_bits;
}

void PageTable::set(unsigned int virtual_address, unsigned int physical_index)
{
    assert(physical_index < num_physical_);
    unsigned int virtual_index = getHighOrder(virtual_address);
    assert(virtual_index < num_virtual_);
    table_[virtual_index].page_frame_number_ = physical_index;
    table_[virtual_index].present_ = true;

    table_[virtual_index].modified_ = false;
    table_[virtual_index].referenced_ = false;
}

PageTable::Entry &PageTable::getEntry(unsigned int address)
{
    unsigned int index = getHighOrder(address);
    assert(index < num_virtual_);
    return table_[index];
}

const PageTable::Entry &PageTable::getEntry(unsigned int address) const
{
    unsigned int index = getHighOrder(address);
    assert(index < num_virtual_);
    return table_[index];
}

void PageTable::initLowOrderMask()
{
    low_order_mask_ = ((unsigned int)1 << low_order_size_) - 1;
}

void PageTable::initHighOrderMask()
{
    // high_order_mask_ = ~(frame_size_-1);
    high_order_mask_ = 0;
    for (size_t i = low_order_size_; i < virtual_address_bits_; i++)
        setNthBit(high_order_mask_, i);
}

void PageTable::setNthBit(unsigned int &number, unsigned int n)
{
    number = ((1 << n) | number);
}

unsigned int PageTable::getHighOrder(unsigned int address) const
{
    return (address & high_order_mask_) >> low_order_size_;
}

unsigned int PageTable::getLowOrder(unsigned int address) const
{
    return address & low_order_mask_;
}

void PageTable::print() const
{
    std::cout << "{ Page Table }\n";
    for (size_t i = 0; i < num_virtual_; i++)
    {
        auto &entry = table_[i];
        std::cout << "\tindex: " << i << "\t[ referenced: " << entry.referenced_;
        std::cout << " modified: " << entry.modified_;
        std::cout << " present: "<<  entry.present_;
        std::cout << " page frame: " << entry.page_frame_number_ << " ]\n";
    }
    
}

PageTable::~PageTable()
{
    delete[] table_;
}

PageTable::Entry::Entry() : referenced_(false), modified_(false), present_(false), page_frame_number_(0)
{
    /** intentionally left blank **/
}

bool PageTable::Entry::isReferenced() const
{
    return referenced_;
}

bool PageTable::Entry::isModified() const
{
    return modified_;
}

unsigned int PageTable::Entry::getFrameNumber() const
{
    return page_frame_number_;
}

void PageTable::Entry::setReferenced(bool val)
{
    referenced_ = val;
}
void PageTable::Entry::setModified(bool val)
{
    modified_ = val;
}

void PageTable::Entry::setPresent(bool present)
{
    present_ = present;
}

bool PageTable::Entry::isPresent() const
{
    return present_;
}

#endif