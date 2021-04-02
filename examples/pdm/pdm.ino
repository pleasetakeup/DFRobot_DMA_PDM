#include "DFRobot_DMA_PDM.h"
#include <SD.h>

typedef  struct
{
  uint32_t ChunkID;        ///< chunk id;"RIFF", 0X46464952
  uint32_t ChunkSize ;       ///< filesize-8
  uint32_t Format;         ///< WAVE,0X45564157
} ChunkRIFF __attribute((aligned (1)));

//< fmt block
typedef  struct
{
  uint32_t ChunkID;        ///< chunk id: "fmt " equal 0X20746D66
  uint32_t ChunkSize ;       ///< (exclude ID and Size); it is 20 here.
  uint16_t AudioFormat;      ///< 0X01,line PCM; 0X11 IMA ADPCM
  uint16_t NumOfChannels;    ///<  2 strevo   1 mono
  uint32_t SampleRate;     ///< ex. 0X1F40 = 8Khz
  uint32_t ByteRate;
  uint16_t BlockAlign;     //
  uint16_t BitsPerSample;    //
  //u16 ByteExtraData;    //no this parameter for line pcm
} ChunkFMT __attribute((aligned (1)));

///< fact block
typedef  struct
{
  uint32_t ChunkID;        ///< chunk id; "fact" equal 0X74636166;
  uint32_t ChunkSize ;       ///< (exclule ID and Size); it is 4 here.
  uint32_t NumOfSamples;
} ChunkFACT __attribute((aligned (1)));

//LIST block
typedef  struct
{
  uint32_t ChunkID;        //chunk id: "LIST" equal 0X74636166;
  uint32_t ChunkSize ;       //(exclude ID and Size);it is 4 here.
} ChunkLIST __attribute((aligned (1)));

//data block
typedef  struct
{
  uint32_t ChunkID;        //chunk id: "data" equal 0X61746164
  uint32_t ChunkSize ;       //(exclude ID and Size)
} ChunkDATA __attribute((aligned (1)));

//wav header
typedef  struct
{
  ChunkRIFF riff; //riff block
  ChunkFMT fmt;   //fmt block
  //  ChunkFACT fact; //fact block. no this block for line PCM
  ChunkDATA data; //data block
} __WaveHeader __attribute((aligned (1)));
uint32_t adcData[1024] = {0};
uint8_t  wavData[512] = {0};
SDFile myFile;
__WaveHeader  header;
void wavFileInit()
{


  //riff block
  header.riff = {
    0X46464952,//u32 ChunkID;     ///< chunk id;"RIFF", 0X46464952
    0,//u32 ChunkSize ;     ///< filesize-8
    0X45564157//u32 Format;       ///< WAVE,0X45564157

  };

  //fmt block
  header.fmt = {
    0x20746D66,//u32 ChunkID;     ///< chunk id: "fmt " equal 0X20746D66
    0x0010, //u32 ChunkSize ;     ///< (exclude ID and Size); it is 16 here.
    0x0001, //u16 AudioFormat;    ///< 0X01,line PCM; 0X11 IMA ADPCM
    0x0001, //u16 NumOfChannels;    ///<  2 strevo   1 mono
    0x00005014/2,//0x00002BC0,//u32 SampleRate;     ///< ex. 0X2BC0 = 10000hz
    0x0000A028/2,//0x0000BB80,//u32 ByteRate;
    0x0002,// u16 BlockAlign;     //
    0x0010 // u16 BitsPerSample;    //2bytes = 16bits
    //0x0000//  u16 ByteExtraData;    //no this parameter for line pcm
  };

  //data block
  header.data = {
    0x61746164, //u32 ChunkID;        //chunk id: "data" equal 0X61746164
    0x00000000  //u32 ChunkSize ;       //(exclude ID and Size)
  };
}
uint16_t sincfilter[64] = {0, 2, 9, 21, 39, 63, 94, 132, 179, 236, 302, 379, 467, 565, 674, 792, 920, 1055, 1196, 1341, 1487, 1633, 1776, 1913, 2042, 2159, 2263, 2352, 2422, 2474, 2506, 2516, 2506, 2474, 2422, 2352, 2263, 2159, 2042, 1913, 1776, 1633, 1487, 1341, 1196, 1055, 920, 792, 674, 565, 467, 379, 302, 236, 179, 132, 94, 63, 39, 21, 9, 2, 0, 0};
uint32_t dataSize;
uint8_t lastData;
uint8_t nowData;
uint16_t runningsum = 0;
uint16_t *sinc_ptr = sincfilter;
void setup() {
  // put your setup code here, to run once:
  Serial.begin(9600);
  while (!Serial);
if (!SD.begin(/*csPin = */3, /*type = */TYPE_NONBOARD_SD_MOUDLE)) {
      SerialUSB.println("initialization failed!");
    while (1);
   }
  //pinMode(A4, INPUT);
  DMAIIS.begin(1, 2);


  DMAIIS.configure(11000 * 4, true);


   myFile = SD.open("rec121.wav", FILE_WRITE | FILE_READ);
   if (myFile) {
    SerialUSB.println("START");
   wavFileInit();
   myFile.write((const char *)&header, sizeof(__WaveHeader));


  //SerialUSB.println(millis() - timr);
  uint32_t timr = millis();
  
  while (millis() - timr < 20000) {
    if (nowData == 0) {
      DMAIIS.start(adcData, 1024);
      nowData = 1;
    }
    if (lastData == 1) {
      uint8_t j = 0;
      uint16_t a = 0;
      for (int16_t i = 0 ; i < 1024 ; i++) {
        uint16_t sample = adcData[i] >> 16 ;
        for (int8_t b = 0; b < 16; b++)
        {
          // start at the LSB which is the 'first' bit to come down the line, chronologically
          // (Note we had to set I2S_SERCTRL_BITREV to get this to work, but saves us time!)
          if (sample & 0x1) {
            runningsum += *sinc_ptr;     // do the convolution
          }
          sinc_ptr++;
          sample >>= 1;
        }
        j++;
        if (j == 4) {
          uint16_t  adc = runningsum / 16;
          wavData[a++] = adc & 0xFF;
          wavData[a++] = adc >> 8;

          sinc_ptr = sincfilter;
          j = 0;
          runningsum = 0;
        }
      }


      lastData = 0;
      myFile.write(wavData, 512);
      dataSize = dataSize + 512;
    }

    if ( !DMAIIS.checkFlag()) {
      lastData = 1;
      nowData = 0;
    }
  }
  myFile.seek(0);
  SerialUSB.println(dataSize);
  __WaveHeader *wavhead = (__WaveHeader *)malloc(sizeof(__WaveHeader));
  myFile.read(wavhead, sizeof(__WaveHeader));
  wavhead->riff.ChunkSize = dataSize + 36;
  wavhead->data.ChunkSize = dataSize;
  myFile.seek(0);
  myFile.write((const char *)wavhead, sizeof(__WaveHeader));
  myFile.close();
}

SerialUSB.println("OVER");

}

void loop() {
  // put your main code here, to run repeatedly:


  // SerialUSB.println(DMAIIS.read());
  delay(10);
}
