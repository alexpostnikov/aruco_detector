#ifndef SERIALUNIX_H
#define SERIALUNIX_H
#include <stdint.h>
#include <string>
#define UNIX
#ifdef UNIX
#include <SerialStream.h>
#include <SerialPort.h>



using namespace LibSerial;


class SerialUnix
{
private:
    SerialStream *mySerialStream;
    // uint8_t addres[3];
    // uint8_t addresSend[3];
    // int countRecivedByte = 0;
    // uint8_t recivedBuff[1000];
    // enum StatesMach {checkAddr=0,getLengthPacket,getBuf,checkCRC,copyBuffer};
    // uint8_t lengthPacket;
    // StatesMach states;
    // uint16_t countSendedData=0;
    // enum StatesSend {sendAddr=0,sendLength,sendPacketSt,sendCRC};
    // uint8_t addrSendCount;
    // StatesSend statesSend;
    // bool finished;
public:

    SerialUnix(std::string &temp)
{


    this->mySerialStream = new SerialStream();
    mySerialStream->Open(temp);
    mySerialStream->SetBaudRate( SerialStreamBuf::BAUD_115200 ) ;
    mySerialStream->SetCharSize( SerialStreamBuf::CHAR_SIZE_8 ) ;
    mySerialStream->SetNumOfStopBits(1) ;
    mySerialStream->SetParity( SerialStreamBuf::PARITY_ODD ) ;
    mySerialStream->SetFlowControl( SerialStreamBuf::FLOW_CONTROL_HARD);
}

    void sendPacket()
    {
        this->mySerialStream ->write("a",1);
    }

    
};



#endif
#endif // SERIALUNIX_H
