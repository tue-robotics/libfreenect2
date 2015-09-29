#include "libfreenect2/depth_packet_processor.h"

#include "timer.h"

#include <iostream>
#include <fstream>

//#include <opencv2/highgui/highgui.hpp>

// ----------------------------------------------------------------------------------------------------

class DepthFileReader
{

public:

    DepthFileReader() : p0_table_(0)
    {
    }

    ~DepthFileReader()
    {        
        file_in_.close();
        delete p0_table_;
    }

    bool open(const std::string& filename)
    {
        file_in_.open(filename.c_str());

        if (!file_in_.is_open())
            return false;

        char sig[4];
        file_in_.read(sig, 4);

        if (file_in_.eof() || sig[0] != 'k' || sig[1] != 'c' || sig[2] != 't' || sig[3] != '2')
        {
            std::cerr << "Unknown file type" << std::endl;
            return false;
        }

        file_in_.read(reinterpret_cast<char*>(&file_version_), sizeof(file_version_));

        if (file_version_ > 1)
        {
            std::cerr << "Unknown file version: " << file_version_ << std::endl;
            return false;
        }

        // - - - - - - - - - - - - - - -
        // Read p0 table

        delete p0_table_;

        file_in_.read(reinterpret_cast<char*>(&p0_table_size_), sizeof(p0_table_size_));

        p0_table_ = new unsigned char[p0_table_size_];
        file_in_.read(reinterpret_cast<char*>(p0_table_), p0_table_size_);

        std::cout << "P0 table: " << p0_table_size_ << std::endl;

        return true;
    }

    bool nextPacket(libfreenect2::DepthPacket& packet)
    {
        uint32_t time, seq, size;

        file_in_.read(reinterpret_cast<char*>(&time), sizeof(time));
        file_in_.read(reinterpret_cast<char*>(&seq), sizeof(seq));
        file_in_.read(reinterpret_cast<char*>(&size), sizeof(size));

        if (file_in_.eof())
            return false;

        packet.timestamp = time;
        packet.sequence = seq;
        packet.buffer_length = size;

        if (!packet.buffer || size > packet.buffer_length)
        {
            delete packet.buffer;
            packet.buffer = new unsigned char[size];
        }

        file_in_.read(reinterpret_cast<char*>(packet.buffer), size);  // write data

        return true;
    }

    unsigned char* p0_table() { return p0_table_; }

    uint32_t p0_table_size() const { return p0_table_size_; }

private:

    uint32_t file_version_;

    std::ifstream file_in_;

    unsigned char* p0_table_;

    uint32_t p0_table_size_;

};

// ----------------------------------------------------------------------------------------------------

struct Listener : public libfreenect2::FrameListener
{

    libfreenect2::Frame* depth_frame;

    bool onNewFrame(libfreenect2::Frame::Type type, libfreenect2::Frame *frame)
    {
        if (type == libfreenect2::Frame::Depth)
            depth_frame = frame;

        return true;
    }
};

// ----------------------------------------------------------------------------------------------------

int main(int argc, char **argv)
{
    if (argc < 2)
    {
        std::cerr << "Please provide depth data filename" << std::endl;
        return 1;
    }

    DepthFileReader reader;
    if (!reader.open(argv[1]))
    {
        std::cerr << "Could not open file" << std::endl;
        return 1;
    }

    Listener listener;

    libfreenect2::CpuDepthPacketProcessor processor_orig;
    processor_orig.load11To16LutFromFile("11to16.bin");
    processor_orig.loadXTableFromFile("xTable.bin");
    processor_orig.loadZTableFromFile("zTable.bin");
    processor_orig.loadP0TablesFromCommandResponse(reader.p0_table(), reader.p0_table_size());
    processor_orig.setFrameListener(&listener);

    libfreenect2::fast::CpuDepthPacketProcessor processor_fast;
    processor_fast.load11To16LutFromFile("11to16.bin");
    processor_fast.loadXTableFromFile("xTable.bin");
    processor_fast.loadZTableFromFile("zTable.bin");
    processor_fast.loadP0TablesFromCommandResponse(reader.p0_table(), reader.p0_table_size());
    processor_fast.setFrameListener(&listener);

    libfreenect2::DepthPacket packet;
    packet.buffer = 0;

    while (reader.nextPacket(packet))
    {       
        Timer t;
        processor_orig.process(packet);
        std::cout << "Original processing took: " << t.getElapsedTimeInMilliSec() << " ms" << std::endl;

        Timer t2;
        processor_fast.process(packet);
        std::cout << "Fast processing took:     " << t2.getElapsedTimeInMilliSec() << " ms" << std::endl;

        std::cout << std::endl;

//        cv::Mat canvas(listener.depth_frame->height, listener.depth_frame->width, CV_32FC1, listener.depth_frame->data);
//        cv::imshow("depth", canvas / 5000);
//        cv::waitKey();
    }

    // Make sure we clean up the packet buffer
    delete packet.buffer;

    return 0;
}
