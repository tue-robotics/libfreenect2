#include "libfreenect2/depth_packet_processor.h"

#include <iostream>
#include <fstream>

// ----------------------------------------------------------------------------------------------------

class DepthFileReader
{

public:

    ~DepthFileReader()
    {
        file_in_.close();
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

        return true;
    }

    bool nextPacket(libfreenect2::DepthPacket& packet)
    {
        uint32_t time = packet.timestamp;
        uint32_t seq = packet.sequence;
        uint32_t size = packet.buffer_length;

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

private:

    uint32_t file_version_;

    std::ifstream file_in_;

};

// ----------------------------------------------------------------------------------------------------

int main(int argc, char **argv)
{
    if (argc < 2)
    {
        std::cerr << "Please provide depth data filename" << std::endl;
        return 1;
    }

    libfreenect2::CpuDepthPacketProcessor processor_orig;
    processor_orig.load11To16LutFromFile("11to16.bin");
    processor_orig.loadXTableFromFile("xTable.bin");
    processor_orig.loadZTableFromFile("zTable.bin");

    libfreenect2::fast::CpuDepthPacketProcessor processor_fast;
    processor_fast.load11To16LutFromFile("11to16.bin");
    processor_fast.loadXTableFromFile("xTable.bin");
    processor_fast.loadZTableFromFile("zTable.bin");


    DepthFileReader reader;
    if (!reader.open(argv[1]))
    {
        std::cerr << "Could not open file" << std::endl;
        return 1;
    }

    libfreenect2::DepthPacket packet;
    packet.buffer = 0;

    while (reader.nextPacket(packet))
    {
        std::cout << packet.sequence << " " << packet.timestamp << " " << packet.buffer_length << std::endl;
    }

    // Make sure we clean up the packet buffer
    delete packet.buffer;

    return 0;
}
