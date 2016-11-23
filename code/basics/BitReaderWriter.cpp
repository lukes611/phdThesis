#include "BitReaderWriter.h"

namespace BIT_MANIPULATOR
{
	//bit reader
	BitReader::BitReader(const char* fname)
	{
		data = NULL;
		dataSize = 0;
		resetPointers();
		readFile(fname);
	}
	BitReader::BitReader(std::string fname)
	{
		data = NULL;
		dataSize = 0;
		resetPointers();
		readFile(fname.c_str());
	}
	BitReader::BitReader(unsigned char * dataIn, int lengthIn)
	{
		data = NULL;
		dataSize = 0;
		resetPointers();
		dataSize = lengthIn;
		data = new LByte[dataSize];
		for(int i = 0; i < dataSize; i++)
		{
			data[i] = dataIn[i];
		}
	}
	void BitReader::error(char* err_name)
	{
		std::cout << err_name << std::endl;
		system("PAUSE");
		exit(1);
	}
	BitReader::~BitReader()
	{
		if(data != NULL)
		{
			delete [] data;
		}
	}
	void BitReader::readFile(const char* fname)
	{
		FILE* fi = fopen(fname, "rb");
		//fopen_s(&fi, fname, "rb");
		if(fi == NULL)
		{
			error("error in BitReader.readFile(), FILE COULD NOT BE READ");
		}
		std::vector<LByte> list;
		while(1)
		{
			unsigned char buffer[51];
			int numRead = fread(buffer, sizeof(unsigned char), 50, fi);
			for(int i = 0; i < numRead; i++)
			{
				list.push_back(buffer[i]);
			}
			if(feof(fi))
			{
				break;
			}

		}
		fclose(fi);
		dataSize = list.size();
		data = new LByte[dataSize];
		for(int i = 0; i < dataSize; i++)
		{
			data[i] = list.at(i);
		}
	}
	void BitReader::resetPointers()
	{
		bitPointer = 0;
		bytePointer = 0;
	}
	int BitReader::pointersAtEnd()
	{
		if(bytePointer == dataSize)
		{
			return 1;
		}
		return 0;
	}
	LBit BitReader::readBit()
	{
		if(bytePointer == dataSize)
		{
			//std::cout << "Reached End of File\n";
			return 0;
		}
		LBit rv = getBit(data[bytePointer], bitPointer);
		bitPointer++;
		if(bitPointer == 8)
		{
			bitPointer = 0;
			bytePointer++;
		}
		return rv;
	}
	LByte BitReader::readByte()
	{
		LByte rv = 0x00;
		for(int i = 0; i < 8; i++)
		{
			setBit(rv, i, readBit());
		}
		return rv;
	}
	void BitReader::readBytes(int num, LByte* bytes)
	{
		for(int i = 0; i < num; i++)
		{
			bytes[i] = readByte();
		}
	}
	LBit BitReader::getBit(LByte in, int at)
	{
		//at = 7 - at;
		if(at < 0 || at > 7)
		{
			error("ERROR - in BitReader.getBit(), index out of range");
			return 0;
		}
		in >>= at;
		in = in & 0xFF;
		if(in & 0x01){
			return 1;
		}else{
			return 0;
		}
	}
	void BitReader::setBit(LByte& input, int location, LBit value)
	{
		if(location < 0 || location > 7)
		{
			error("ERROR - BitReader.setBit(), index not correct");
		}

		LByte mask[] = { 0x01, 0x02, 0x04, 0x08, 0x10, 0x20, 0x40, 0x80 };
		if(value == 1)
		{
			input = input | mask[location];
		}
		else
		{
			input = input & (~mask[location]);
		}
	}
	void BitReader::print(LByte byte)
	{
		for(int i = 0; i < 8; i++)
		{
			std::cout << getBit(byte, i);
		}
		std::cout << std::endl;
	}
	void BitReader::print(LByte byte, int index)
	{
		std::cout << getBit(byte, index) << std::endl;
	}
	void BitReader::print(int num)	//prints all bytes read by this construct up to num
	{
		int savedBitPointer = bitPointer;
		int savedBytePointer = bytePointer;
		resetPointers();
		for(int i = 0; i < num && i < dataSize; i++)
		{
			LByte byte = readByte();
			print(byte);
		}
		bitPointer = savedBitPointer;
		bytePointer = savedBytePointer;
	}
	void BitReader::saveRaw(char* fname)
	{
		int savedBitPointer = bitPointer;
		int savedBytePointer = bytePointer;
		resetPointers();
		FILE* fi = fopen(fname, "wb");
		//fopen_s(&fi, fname, "wb");
		for(int i = 0; i < dataSize; i++)
		{
			LByte byte = readByte();
			fwrite(&byte, sizeof(unsigned char), 1, fi);
		}
		fclose(fi);
		bitPointer = savedBitPointer;
		bytePointer = savedBytePointer;
	}
	void BitReader::flipBits(LByte& c)
	{
		for(int i = 0; i < 8; i++)
		{
			if(getBit(c, i) == 1)
			{
				setBit(c, i, 0);
			}
			else
			{
				setBit(c, i, 1);
			}
		}
	}


	//bit writer
	BitWriter::BitWriter()
	{
		work_on = 0x00;
		bitPointer = 0;
		dataSize = 0;
		data = new std::vector<LByte>;
	}
	BitWriter::~BitWriter()
	{
		delete data;
	}
	void BitWriter::error(char* err_name)
	{
		std::cout << err_name << std::endl;
		system("PAUSE");
		exit(1);
	}
	LBit BitWriter::getBit(LByte in, int at)
	{
		//at = 7 - at;
		if(at < 0 || at > 7)
		{
			error("ERROR - in BitWriter.getBit(), index out of range");
			return 0;
		}
		in >>= at;
		in = in & 0xFF;
		if(in & 0x01){
			return 1;
		}else{
			return 0;
		}
	}
	void BitWriter::setBit(LByte& input, int location, LBit value)
	{
		if(location < 0 || location > 7)
		{
			error("ERROR - BitWriter.setBit(), index not correct");
		}

		LByte mask[] = { 0x01, 0x02, 0x04, 0x08, 0x10, 0x20, 0x40, 0x80 };
		if(value == 1)
		{
			input = input | mask[location];
		}
		else
		{
			input = input & (~mask[location]);
		}
	}
	void BitWriter::addBit(LBit input)
	{
		setBit(work_on, bitPointer, input);
		bitPointer++;
		if(bitPointer >= 8)
		{
			data->push_back(work_on);
			work_on = 0x00;
			bitPointer = 0;
			dataSize++;
		}

	}
	void BitWriter::addByte(LByte input)
	{
		for(int i = 0; i < 8; i++)
		{
			addBit(getBit(input, i));
		}
	}
	void BitWriter::addBytes(int num, unsigned char* bytes)
	{
		for(int i = 0; i < num; i++)
		{
			addByte(bytes[i]);
		}
	}

	void BitWriter::operator+=(LBit bit_in)
	{
		addBit(bit_in);
	}

	void BitWriter::operator+=(LByte byte_in)
	{
		addByte(byte_in);
	}

	void BitWriter::save(char* fname)
	{
		FILE* fi = fopen(fname , "wb");
		//fopen_s(&fi, fname, "wb");
		for(int i = 0; i < dataSize; i++)
		{
			LByte byte = data->at(i);
			fwrite(&byte, 1, 1, fi);
		}
		if(bitPointer > 0)
		{
			fwrite(&work_on, 1, 1, fi);
		}
		fclose(fi);
	}

	void BitWriter::save(const char * fname)
	{
		FILE* fi = fopen(fname, "wb");// fopen_s(&fi, fname, "wb");
		for(int i = 0; i < dataSize; i++)
		{
			LByte byte = data->at(i);
			fwrite(&byte, 1, 1, fi);
		}
		if(bitPointer > 0)
		{
			fwrite(&work_on, 1, 1, fi);
		}
		fclose(fi);
	}

	void BitWriter::print()
	{
		for(int i = 0; i < dataSize; i++)
		{
			std::cout << i << ":\t";
			print(data->at(i));
		}
	}
	void BitWriter::print(int num)
	{
		for(int i = 0; i < num && i < dataSize; i++)
		{
			std::cout << i << ":\t";
			print(data->at(i));
		}
	}
	void BitWriter::print(LByte byte)
	{
		for(int i = 0; i < 8; i++)
		{
			std::cout << getBit(byte, i);
		}
		std::cout << std::endl;
	}
	void BitWriter::print(LByte byte, int index)
	{
		std::cout << getBit(byte, index) << std::endl;
	}


}
