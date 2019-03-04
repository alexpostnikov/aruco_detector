#ifdef UNIX
#include "serialunix.h"
#include <string>

#include "/usr/local/include/SerialStream.h"

SerialUnix::SerialUnix(std::string &temp)
{


    this->mySerialStream = new SerialStream();
    mySerialStream->Open(temp);
    mySerialStream->SetBaudRate( SerialStreamBuf::BAUD_115200 ) ;
    mySerialStream->SetCharSize( SerialStreamBuf::CHAR_SIZE_8 ) ;
    mySerialStream->SetNumOfStopBits(1) ;
    mySerialStream->SetParity( SerialStreamBuf::PARITY_ODD ) ;
    mySerialStream->SetFlowControl( SerialStreamBuf::FLOW_CONTROL_HARD);
}

void SerialUnix::sendPacket()
{
    this->mySerialStream ->write("a",1);
}


// int SerialUnix::reciveByte()
// {
//     uint8_t reciveByte=0;
//    while((*mySerialStream) >>reciveByte)
//    {
//     addres[2] = reciveByte;
//     uint16_t crcPack;
//     switch (states)
//     {
//     case checkAddr:
//         if((addres[0] == 0xA) && (addres[1] == 0xB) && (addres[2] == 0xC))
//             states=getLengthPacket;
//         else
//         {
//             addres[0] = addres[1];
//             addres[1] = addres[2];
//         }
//         break;
//     case getLengthPacket:
//         lengthPacket=reciveByte;
//         countRecivedByte=0;
//         states=getBuf;
//         break;
//     case getBuf:
//         recivedBuff[countRecivedByte] = reciveByte;
//         countRecivedByte++;

//         if(countRecivedByte >= lengthPacket+sizeof(crcPack))
//         {
//             states=checkCRC;
//         }
//         else break;
//     case checkCRC:
//         crcPack=CRC16(recivedBuff,lengthPacket);
//         uint8_t firstPart,secondPart;
//         firstPart=recivedBuff[countRecivedByte-2];
//         secondPart=recivedBuff[countRecivedByte-1];
//         if (crcPack==((secondPart<<8)|firstPart))
//             states=copyBuffer;
//         else
//         {
//             states=checkAddr;
//             break;
//         }
//     case copyBuffer:
//         transBuff->getData(recivedBuff,countRecivedByte-2);
//         countRecivedByte = 0;
//         states=checkAddr;
//         return 1;

//     }
//    }
//    return 0;

// //    if((addres[0] == 0xA) && (addres[1] == 0xB) && (addres[2] == 0xC))
// //    {
// //        addresIsTrue = true;  // Все заебись ставим флаг
// //    }
// //    else
// //    {
// //        addres[0] = addres[1];
// //        addres[1] = addres[2];
// //        addresIsTrue = false;
// //    }

// //    if(addresIsTrue == true)
// //    {
// //        if(countRecivedByte == 0)
// //            lengthPacket = reciveByte;
// //
// //        recivedBuff[countRecivedByte] = reciveByte;
// //        countRecivedByte++;
// //
// //        if(countRecivedByte >= lengthPacket+1)
// //        {
// //            //parent->getData(recivedBuff,lengthPacket);
// //            countRecivedByte = 0;
// //            addresIsTrue = false;
// //
// //        }  // Принимаем основной пакет инкрементируем и проверям если
//     }

#endif
