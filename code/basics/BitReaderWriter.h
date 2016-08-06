#ifndef BITREADERWRITER_H
#define BITREADERWRITER_H

#include <stdlib.h>
#include <iostream>
#include <vector>
#include <string>

namespace BIT_MANIPULATOR { class BitReader; class BitWriter; }

namespace BIT_MANIPULATOR
{


	
	//HERE in a byte: bits are in the order [7..0] the 0th index is the bit on the full right, the 7th bit is on the full left

	typedef unsigned char LByte;
	typedef int LBit;

	
	class BitReader
	{
	public:
		LByte* data;
		int dataSize;
		int bitPointer;
		int bytePointer;
	
		BitReader(const char* fname);
		BitReader(std::string fname);
		BitReader(unsigned char * dataIn, int lengthIn);
		void error(char* err_name);
		~BitReader();
		void readFile(const char* fname);
		void resetPointers();
		int pointersAtEnd();
		LBit readBit();
		LByte readByte();
		void readBytes(int num, LByte* bytes);
		LBit getBit(LByte in, int at);
		void setBit(LByte& input, int location, LBit value);
		void print(LByte byte);
		void print(LByte byte, int index);
		void print(int num);	//prints all bytes read by this construct up to num
		void saveRaw(char* fname);
		void flipBits(LByte& c);

	};



	class BitWriter
	{
		LByte work_on;
	public:
		int dataSize;
		int bitPointer;
		std::vector<LByte>* data;	
		BitWriter();
		~BitWriter();
		void error(char* err_name);
		LBit getBit(LByte in, int at);
		void setBit(LByte& input, int location, LBit value);
		void addBit(LBit input);
		void addByte(LByte input);
		void addBytes(int num, unsigned char* bytes);
		void operator+=(LBit bit_in);
		void operator+=(LByte byte_in);
		void save(char* fname);

		void save(const char * fname);
	
		void print();
		void print(int num);
		void print(LByte byte);
		void print(LByte byte, int index);

	};

}


#endif