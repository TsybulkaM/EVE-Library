// EVE Processor Agnostic Library (Condensed)
//
// This library is for the FT812, FT813, BT815, BT816, BT817, BT818
//
// Supported Platforms such as but not limited to:
// Arduino (AVR, SAM, etc.)
// STM32 (Multiple series support)
// NXP (Kinetis, iMX RT, etc.)
// Raspberry Pi (RP2040, RP2350, RP2354)
// Other supported platforms
//
// This "library" consists of the files "eve.c" and "eve.h".
//
// In persuit of the common goal of simplicity and understandability I find that I am unable to
// make function prototypes that match Bridgetek example code.  I draw the line between the
// EVE and all other hardware. The library is "clean" and includes no abstraction at all, unlike
// much of the example code on the Internet which is sort of application and abstraction mixed
// together in a confusing abuse of my eye-holes.
// My intent is to be as straight forward and understandable as possible, so while function
// names and parameter lists are different than Bridgetek code examples, they should be easily
// recognizable.  I have also made every attempt to reference Bridgetek documentation against
// the code to act as a translation to help in understanding.
//
// Notes on the operation of the EVE command processing engine - THE FIFO
//
// First be aware that the FTDI/Bridgetek documentation variously refers to you as "User", "MCU",
// "Host".
//
// The FIFO, like all FIFO's needs pointers to indicate the starting address of buffered data and
// the end address of buffered data.  There is wrapping involved, but the basic idea is clear.
// EVE takes data into it's FIFO using a fully defined write operation to a memory address - that
// is, you need to take care of the wrapping - to you, it is not a FIFO - it is a piece of memory.
// EVE keeps track of it's own read address location, but relies on you to write the address
// of the end of buffered data.
//
// So as commands are loaded into RAM - into the FIFO space - EVE will do nothing in response.
// EVE is happy to take your data and store it for you while it sits with it's read address and
// write address set to the same value.  Once the commands are loaded, the next available address
// is manually written (by you) to the register in which Eve stores the FIFO write pointer
// (REG_CMD_WRITE).
//
// Following this, EVE discovers that the addresses are different and begins processing commands
// while updating it's own read pointer until the read and write pointers are the same.
//
// Be aware that EVE stores only the offset into the "FIFO" as 16 bits, so any use of the offset
// requires adding the base address (RAM_CMD 0x308000) to the resultant 32 bit value.

#include "eve.h"     // Header for this file with prototypes, defines, and typedefs
#include "hw_api.h"  // For SPI abstraction
#include <stdbool.h> // For true/false
#include <stdint.h>  // Find integer types like "uint8_t"
#include <stdio.h>

#define WorkBuffSz 512
#define Log printf

// Global Variables
uint16_t FifoWriteLocation = 0;
char LogBuf[WorkBuffSz]; // The singular universal data array used for all things including logging

const uint8_t Touch_il[] = {
    26,  255, 255, 255, 32,  32,  48,  0,   4,   0,   0,   0,   2,   0,   0,   0,   26,  255, 255,
    255, 0,   176, 48,  0,   4,   0,   0,   0,   119, 2,   0,   0,   34,  255, 255, 255, 0,   176,
    48,  0,   120, 218, 93,  84,  93,  104, 93,  69,  16,  158, 61,  123, 115, 53,  137, 92,  207,
    41,  165, 136, 228, 200, 61,  199, 27,  47,  233, 77,  161, 152, 166, 34,  40,  204, 38,  109,
    105, 19,  66,  17,  65,  243, 32,  116, 247, 220, 244, 254, 215, 16,  250, 96,  145, 104, 167,
    10,  62,  29,  56,  248, 98,  21,  76,  241, 193, 171, 72,  192, 39,  131, 72,  95,  4,   107,
    144, 82,  201, 131, 80,  138, 16,  240, 161, 136, 79,  230, 165, 69,  80,  33,  157, 57,  247,
    182, 136, 44,  179, 51,  231, 155, 157, 157, 111, 118, 103, 207, 91,  30,  0,   80,  218, 80,
    148, 37,  85,  231, 187, 176, 251, 169, 3,   202, 154, 190, 219, 112, 1,   75,  213, 165, 182,
    202, 150, 224, 85,  39,  150, 216, 97,  87,  214, 14,  214, 135, 221, 135, 86,  106, 117, 51,
    109, 140, 97,  218, 224, 29,  90,  190, 203, 90,  186, 149, 54,  88,  175, 66,  39,  109, 204,
    96,  150, 148, 80,  183, 94,  42,  107, 247, 50,  100, 54,  181, 37,  60,  130, 47,  66,  96,
    179, 132, 179, 183, 161, 195, 60,  154, 85,  82,  228, 187, 25,  155, 49,  139, 5,   8,   122,
    190, 43,  96,  218, 43,  226, 56,  45,  226, 34,  156, 69,  201, 165, 19,  223, 5,   221, 124,
    230, 172, 122, 117, 150, 241, 34,  86,  57,  2,   40,  236, 6,   246, 82,  185, 98,  21,  233,
    250, 90,  249, 16,  46,  227, 4,   150, 242, 57,  236, 206, 99,  27,  180, 155, 64,  237, 50,
    87,  194, 131, 84,  71,  169, 232, 0,   107, 169, 98,  25,  15,  209, 235, 168, 109, 140, 79,
    15,  61,  79,  61,  242, 76,  176, 39,  179, 194, 108, 148, 128, 57,  233, 118, 86,  127, 130,
    173, 172, 9,   180, 129, 208, 248, 172, 156, 90,  160, 49,  10,  122, 95,  150, 133, 123, 96,
    191, 41,  227, 7,   125, 60,  137, 95,  115, 181, 208, 217, 132, 204, 245, 177, 15,  211, 116,
    6,   106, 180, 132, 138, 14,  211, 140, 213, 238, 48,  71,  140, 225, 143, 229, 62,  126, 207,
    43,  20,  213, 72,  162, 107, 116, 6,   175, 231, 235, 32,  95,  55,  64,  118, 96,  135, 119,
    184, 137, 91,  248, 29,  203, 207, 240, 139, 204, 195, 175, 155, 176, 141, 125, 188, 142, 178,
    223, 31,  156, 177, 70,  91,  8,   189, 219, 28,  81,  203, 243, 136, 111, 11,  111, 195, 101,
    42,  208, 199, 124, 223, 127, 161, 65,  189, 114, 7,   119, 24,  13,  236, 99,  164, 219, 123,
    172, 139, 67,  61,  50,  212, 133, 161, 214, 67,  237, 13,  181, 202, 245, 30,  222, 197, 40,
    2,   72,  173, 97,  43,  75,  238, 98,  108, 254, 51,  148, 162, 127, 112, 210, 0,   237, 129,
    135, 123, 48,  105, 142, 170, 99,  38,  254, 223, 56,  170, 38,  205, 243, 70,  187, 93,  136,
    233, 85,  40,  179, 84,  88,  34,  190, 235, 19,  166, 132, 167, 204, 179, 252, 229, 161, 135,
    251, 251, 208, 8,   236, 254, 126, 106, 249, 190, 19,  233, 73,  65,  54,  236, 27,  166, 136,
    5,   122, 1,   151, 205, 9,   117, 74,  157, 85,  167, 149, 196, 78,  210, 34,  140, 80,  96,
    161, 115, 206, 156, 196, 11,  81,  106, 243, 154, 86,  215, 35,  185, 65,  189, 166, 87,  222,
    81,  153, 19,  180, 134, 239, 69,  51,  48,  176, 138, 88,  177, 239, 154, 243, 220, 29,  97,
    247, 178, 122, 142, 247, 8,   236, 60,  234, 36,  176, 31,  69,  125, 132, 206, 135, 28,  51,
    205, 232, 17,  74,  123, 163, 44,  69,  22,  104, 72,  159, 241, 153, 42,  67,  210, 1,   37,
    186, 6,   95,  152, 111, 163, 117, 148, 142, 133, 94,  224, 46,  154, 170, 115, 38,  236, 74,
    255, 254, 160, 4,   43,  112, 111, 149, 112, 29,  127, 162, 133, 33,  46,  92,  62,  55,  255,
    130, 226, 19,  58,  102, 30,  39,  189, 150, 37,  64,  55,  204, 182, 217, 229, 78,  185, 161,
    196, 246, 221, 182, 81,  102, 23,  198, 105, 137, 51,  252, 25,  201, 78,  192, 239, 204, 231,
    174, 129, 250, 239, 81,  224, 206, 153, 121, 148, 215, 50,  66,  208, 252, 45,  122, 27,  239,
    113, 220, 56,  93,  195, 123, 209, 43,  102, 193, 156, 6,   57,  49,  189, 50,  197, 249, 158,
    228, 216, 49,  252, 59,  154, 226, 14,  19,  27,  98,  62,  79,  174, 227, 190, 130, 43,  247,
    13,  223, 251, 28,  115, 104, 23,  60,  197, 95,  163, 158, 151, 207, 64,  87,  205, 51,  180,
    201, 124, 174, 154, 233, 43,  155, 112, 199, 124, 98,  184, 246, 102, 28,  135, 158, 79,  183,
    152, 183, 206, 223, 31,  208, 1,   174, 47,  236, 242, 121, 183, 94,  139, 143, 155, 152, 95,
    212, 113, 163, 173, 248, 3,   39,  158, 156, 71,  61,  159, 155, 75,  113, 197, 10,  58,  136,
    17,  159, 174, 15,  230, 175, 204, 0,   227, 28,  73,  145, 95,  175, 182, 83,  204, 65,  91,
    65,  43,  220, 5,   72,  146, 227, 125, 51,  203, 140, 102, 249, 175, 114, 33,  150, 12,  226,
    145, 8,   109, 151, 61,  100, 134, 28,  209, 188, 24,  191, 233, 105, 39,  227, 87,  117, 112,
    110, 98,  78,  113, 21,  151, 188, 7,   73,  25,  126, 203, 0,   26,  255, 255, 255, 32,  32,
    48,  0,   4,   0,   0,   0,   0,   0,   0,   0};

const uint8_t Touch_cyt[] = {
    26,  255, 255, 255, 32,  32,  48,  0,   4,   0,   0,   0,   2,   0,   0,   0,   26,  255, 255,
    255, 0,   176, 48,  0,   4,   0,   0,   0,   82,  2,   0,   0,   34,  255, 255, 255, 0,   176,
    48,  0,   120, 218, 93,  83,  65,  104, 156, 69,  20,  126, 243, 207, 102, 109, 19,  221, 238,
    47,  82,  138, 100, 233, 255, 111, 55,  93,  218, 77,  165, 52,  91,  68,  80,  120, 47,  73,
    75,  155, 16,  74,  205, 193, 28,  10,  157, 249, 55,  238, 38,  187, 43,  33,  120, 17,  137,
    246, 85,  193, 211, 15,  63,  189, 180, 30,  76,  233, 193, 85,  36,  224, 169, 161, 72,  47,
    130, 109, 40,  45,  37,  130, 80,  60,  52,  158, 114, 16,  47,  1,   47,  182, 40,  174, 111,
    254, 252, 17,  145, 225, 205, 123, 243, 205, 123, 243, 222, 204, 251, 230, 109, 15,  0,   56,
    110, 42,  78,  162, 170, 45,  218, 82,  231, 11,  11,  156, 180, 138, 118, 213, 250, 34,  85,
    27,  155, 170, 88,  14,  175, 90,  103, 57,  187, 212, 113, 190, 187, 254, 165, 206, 158, 21,
    27,  221, 138, 155, 131, 24,  55,  229, 132, 133, 162, 77,  22,  244, 66,  220, 20,  189, 4,
    237, 184, 57,  134, 73,  84,  64,  189, 240, 102, 160, 237, 91,  144, 152, 216, 20,  240, 4,
    190, 1,   190, 73,  34,  201, 190, 8,   109, 169, 163, 85,  101, 197, 69,  59,  102, 18,  169,
    98,  10,  252, 110, 209, 230, 48,  238, 230, 113, 136, 167, 113, 26,  46,  160, 203, 165, 163,
    162, 245, 59,  233, 44,  89,  245, 210, 105, 193, 243, 88,  149, 8,   224, 82,  199, 55,  31,
    4,   21,  163, 88,  55,  150, 131, 131, 56,  135, 195, 88,  72,  231, 82,  103, 2,   23,  65,
    219, 97,  212, 54,  177, 5,   124, 133, 27,  232, 110, 244, 178, 104, 119, 139, 57,  60,  200,
    239, 160, 54,  101, 124, 53,  219, 57,  244, 239, 206, 176, 236, 36,  198, 85,  182, 159, 65,
    106, 210, 139, 73,  227, 69,  177, 146, 22,  240, 42,  66,  243, 86,  16,  27,  224, 65,  246,
    187, 95,  7,   174, 118, 223, 220, 14,  240, 179, 30,  158, 193, 111, 229, 182, 208, 94,  131,
    196, 246, 176, 7,   163, 124, 30,  106, 60,  131, 138, 143, 243, 152, 209, 246, 184, 68,  12,
    226, 253, 160, 135, 223, 139, 135, 226, 26,  187, 232, 26,  159, 199, 187, 169, 31,  164, 126,
    187, 200, 38,  108, 202, 9,   15,  113, 29,  191, 19,  121, 12,  63,  185, 57,  91,  61,  132,
    13,  236, 225, 93,  116, 231, 253, 42,  25,  107, 188, 142, 208, 125, 34,  17,  181, 52,  143,
    219, 91,  199, 39,  112, 133, 115, 252, 185, 244, 251, 15,  36,  212, 243, 63,  227, 166, 160,
    190, 121, 129, 245, 226, 142, 232, 124, 166, 7,   50,  157, 203, 180, 206, 180, 151, 105, 149,
    234, 29,  220, 198, 48,  4,   136, 13,  137, 149, 68,  219, 88,  166, 255, 12,  165, 248, 79,
    28,  33,  224, 29,  240, 112, 7,   70,  232, 164, 170, 83,  249, 127, 227, 164, 26,  161, 83,
    164, 237, 22,  148, 121, 22,  2,   145, 138, 72,  40,  189, 158, 164, 2,   158, 165, 35,  178,
    242, 208, 195, 126, 31,  154, 190, 233, 247, 99,  35,  253, 142, 28,  39,  29,  178, 106, 46,
    81,  30,  115, 252, 58,  206, 209, 164, 58,  171, 46,  168, 115, 202, 197, 142, 240, 52,  12,
    176, 111, 160, 125, 153, 206, 224, 123, 97,  108, 210, 59,  45,  173, 132, 174, 131, 122, 89,
    207, 127, 164, 18,  235, 208, 26,  126, 18,  142, 193, 174, 149, 199, 138, 249, 152, 222, 21,
    118, 148, 58,  87,  212, 81,  57,  195, 55,  19,  168, 35,  223, 92,  15,  123, 8,   237, 107,
    18,  51,  42,  232, 9,   142, 187, 251, 69,  242, 34,  208, 116, 60,  147, 55,  85,  21,  118,
    12,  40,  240, 77,  248, 138, 238, 132, 43,  232, 24,  11,  93,  223, 190, 79,  85,  107, 169,
    212, 113, 252, 253, 65,  57,  44,  39,  220, 42,  224, 10,  62,  224, 169, 12,  119, 181, 124,
    73,  127, 73,  190, 125, 242, 178, 138, 148, 188, 84,  157, 196, 94,  78,  34,  224, 123, 180,
    65,  91,  194, 152, 123, 202, 217, 69,  187, 65,  138, 182, 96,  136, 103, 36,  211, 239, 161,
    59,  17,  228, 191, 21,  133, 61,  208, 248, 45,  244, 237, 101, 154, 64,  247, 107, 6,   24,
    90,  219, 225, 135, 248, 76,  226, 134, 248, 38,  62,  11,  47,  210, 20,  157, 3,   247, 114,
    122, 254, 152, 228, 61,  32,  177, 131, 248, 119, 120, 76,  152, 230, 236, 92,  89,  222, 85,
    238, 243, 92,  193, 213, 231, 36,  213, 140, 187, 122, 246, 121, 74,  86,  47,  121, 94,  58,
    3,   223, 160, 83,  188, 38,  245, 220, 160, 209, 171, 107, 240, 148, 224, 211, 31,  233, 145,
    244, 176, 46,  178, 171, 133, 79,  13,  183, 170, 72,  183, 102, 202, 245, 20,  59,  44,  216,
    35,  234, 39,  123, 243, 36,  235, 232, 27,  170, 103, 113, 123, 226, 34,  20,  159, 150, 95,
    248, 154, 167, 37,  11,  10,  246, 139, 58,  52,  30,  140, 43,  89,  205, 122, 255, 0,   96,
    45,  107, 233, 26,  255, 255, 255, 32,  32,  48,  0,   4,   0,   0,   0,   0,   0,   0,   0};

static uint32_t Width;
static uint32_t Height;
static uint32_t HOffset;
static uint32_t VOffset;
static uint8_t Touch;
void Calibrate_Fixed(uint32_t width_pixels,
                     uint32_t height_pixels,
                     uint32_t touch_x_max,
                     uint32_t touch_y_max);

uint32_t Display_Width()
{
  return Width;
}

uint32_t Display_Height()
{
  return Height;
}

uint8_t Display_Touch()
{
  return Touch;
}

uint32_t Display_HOffset()
{
  return HOffset;
}
uint32_t Display_VOffset()
{
  return VOffset;
}

#define COMMAND 0
#define DATA 1
#define CS_ENABLE 0
#define CS_DISABLE 1

#define CS 0x02
#define SCL 0x04
#define SDA 0x08

void GPIOX_WriteBit(uint8_t data, bool state)
{
  if (state)
  {
    wr8(REG_GPIOX + RAM_REG, (rd8(REG_GPIOX + RAM_REG)) | data);
  }
  else
  {
    data = ~data;
    wr8(REG_GPIOX + RAM_REG, (rd8(REG_GPIOX + RAM_REG)) & (data));
  }
}

void MO_SPIBB_CS(uint8_t enable)
{
  wr16(REG_GPIOX_DIR + RAM_REG, 0x00f7); // set SDA GPIO0 as output

  switch (enable)
  {
  case 0:
    wr8(REG_GPIOX + RAM_REG, (rd8(REG_GPIOX + RAM_REG)) & ~CS);
    break;
  case 1:
    wr8(REG_GPIOX + RAM_REG, (rd8(REG_GPIOX + RAM_REG)) | CS);
    break;
  }
}

void MO_SPIBB_Send(bool type, uint8_t data)
{
  unsigned char m = 0x80;
  uint8_t i;

  wr16(REG_GPIOX_DIR + RAM_REG, 0x80ff);
  wr16(REG_GPIOX + RAM_REG, 0x80f0);

  if (type == COMMAND)
  {

    GPIOX_WriteBit(SCL, 0);
    GPIOX_WriteBit(SDA, 0);
    GPIOX_WriteBit(SCL, 1);
  }
  else if (type == DATA)
  {

    GPIOX_WriteBit(SCL, 0);
    GPIOX_WriteBit(SDA, 1);
    GPIOX_WriteBit(SCL, 1);
  }

  for (i = 0; i < 8; i++)
  {
    GPIOX_WriteBit(SCL, 0);
    if (data & m)
    {
      GPIOX_WriteBit(SDA, 1);
    }
    else
    {
      GPIOX_WriteBit(SDA, 0);
    }
    GPIOX_WriteBit(SCL, 1);
    m = m >> 1;
  }
  GPIOX_WriteBit(SCL, 0);
}

void MO_ST7789V_init(void)
{
  wr16(REG_GPIOX_DIR + RAM_REG, (0x00FF));
  wr16(REG_GPIOX + RAM_REG, 0x00F7);

  HAL_Delay(100); // 1000

  // the following is from AFY240320A0-2.8INTH data sheet, page 25
  MO_SPIBB_CS(CS_ENABLE);
  MO_SPIBB_Send(COMMAND, 0x11);
  MO_SPIBB_CS(CS_DISABLE);
  HAL_Delay(120); // Delay 120ms

  MO_SPIBB_CS(CS_ENABLE);
  MO_SPIBB_Send(COMMAND, 0x36); // MADCTRL
  MO_SPIBB_Send(DATA, 0x00);    // was 0x80
  MO_SPIBB_CS(CS_DISABLE);

  MO_SPIBB_CS(CS_ENABLE);
  MO_SPIBB_Send(COMMAND, 0x3a);
  MO_SPIBB_Send(DATA, 0x66);
  MO_SPIBB_CS(CS_DISABLE);

  MO_SPIBB_CS(CS_ENABLE);
  MO_SPIBB_Send(COMMAND, 0xB0);
  MO_SPIBB_Send(DATA, 0x12); // <<-- RGB interface
  MO_SPIBB_Send(DATA, 0x00);
  MO_SPIBB_CS(CS_DISABLE);

  MO_SPIBB_CS(CS_ENABLE);
  MO_SPIBB_Send(COMMAND, 0x21);
  MO_SPIBB_CS(CS_DISABLE);

  //--------------------------------ST7789V Frame rate setting----------------------------------//
  MO_SPIBB_CS(CS_ENABLE);
  MO_SPIBB_Send(COMMAND, 0xb2);
  MO_SPIBB_Send(DATA, 0x0c);
  MO_SPIBB_Send(DATA, 0x0c);
  MO_SPIBB_Send(DATA, 0x00);
  MO_SPIBB_Send(DATA, 0x33);
  MO_SPIBB_Send(DATA, 0x33);
  MO_SPIBB_CS(CS_DISABLE);

  MO_SPIBB_CS(CS_ENABLE);
  MO_SPIBB_Send(COMMAND, 0xb7);
  MO_SPIBB_Send(DATA, 0x35);
  MO_SPIBB_CS(CS_DISABLE);
  //---------------------------------ST7789V Power setting--------------------------------------//
  MO_SPIBB_CS(CS_ENABLE);
  MO_SPIBB_Send(COMMAND, 0xbb);
  MO_SPIBB_Send(DATA, 0x18); // 1F
  MO_SPIBB_CS(CS_DISABLE);

  MO_SPIBB_CS(CS_ENABLE);
  MO_SPIBB_Send(COMMAND, 0xc0);
  MO_SPIBB_Send(DATA, 0x2c);
  MO_SPIBB_CS(CS_DISABLE);

  MO_SPIBB_CS(CS_ENABLE);
  MO_SPIBB_Send(COMMAND, 0xc2);
  MO_SPIBB_Send(DATA, 0x01);
  MO_SPIBB_Send(DATA, 0xFF);
  MO_SPIBB_CS(CS_DISABLE);

  MO_SPIBB_CS(CS_ENABLE);
  MO_SPIBB_Send(COMMAND, 0xc3);
  MO_SPIBB_Send(DATA, 0x20); // 12
  MO_SPIBB_CS(CS_DISABLE);

  MO_SPIBB_CS(CS_ENABLE);
  MO_SPIBB_Send(COMMAND, 0xc4);
  MO_SPIBB_Send(DATA, 0x20);
  MO_SPIBB_CS(CS_DISABLE);

  MO_SPIBB_CS(CS_ENABLE);
  MO_SPIBB_Send(COMMAND, 0xc6);
  MO_SPIBB_Send(DATA, 0x0f);
  MO_SPIBB_CS(CS_DISABLE);

  MO_SPIBB_CS(CS_ENABLE);
  MO_SPIBB_Send(COMMAND, 0xd0);
  MO_SPIBB_Send(DATA, 0xa4);
  MO_SPIBB_Send(DATA, 0xa1);
  MO_SPIBB_CS(CS_DISABLE);
  //--------------------------------ST7789V gamma setting--------------------------------------//

  MO_SPIBB_CS(CS_ENABLE);
  MO_SPIBB_Send(COMMAND, 0xe0);
  MO_SPIBB_Send(DATA, 0xd0);
  MO_SPIBB_Send(DATA, 0x08);
  MO_SPIBB_Send(DATA, 0x11);
  MO_SPIBB_Send(DATA, 0x08);
  MO_SPIBB_Send(DATA, 0x0c);
  MO_SPIBB_Send(DATA, 0x15);
  MO_SPIBB_Send(DATA, 0x39);
  MO_SPIBB_Send(DATA, 0x33);
  MO_SPIBB_Send(DATA, 0x50);
  MO_SPIBB_Send(DATA, 0x36);
  MO_SPIBB_Send(DATA, 0x13);
  MO_SPIBB_Send(DATA, 0x14);
  MO_SPIBB_Send(DATA, 0x29);
  MO_SPIBB_Send(DATA, 0x2d);
  MO_SPIBB_CS(CS_DISABLE);

  MO_SPIBB_CS(CS_ENABLE);
  MO_SPIBB_Send(COMMAND, 0xe1);
  MO_SPIBB_Send(DATA, 0xd0);
  MO_SPIBB_Send(DATA, 0x08);
  MO_SPIBB_Send(DATA, 0x10);
  MO_SPIBB_Send(DATA, 0x08);
  MO_SPIBB_Send(DATA, 0x06);
  MO_SPIBB_Send(DATA, 0x06);
  MO_SPIBB_Send(DATA, 0x39);
  MO_SPIBB_Send(DATA, 0x44);
  MO_SPIBB_Send(DATA, 0x51);
  MO_SPIBB_Send(DATA, 0x0b);
  MO_SPIBB_Send(DATA, 0x16);
  MO_SPIBB_Send(DATA, 0x14);
  MO_SPIBB_Send(DATA, 0x2f);
  MO_SPIBB_Send(DATA, 0x31);
  MO_SPIBB_CS(CS_DISABLE);

  MO_SPIBB_CS(CS_ENABLE);
  MO_SPIBB_Send(COMMAND, 0x29);
  MO_SPIBB_CS(CS_DISABLE);
}

// Call this function once at powerup to reset and initialize the EVE chip
// The Display, board and touch defines can be found in displays.h.
int EVE_Init(int display, int board, int touch)
{
  uint32_t Ready = false;
  int DWIDTH;
  int DHEIGHT;
  int PIXVOFFSET;
  int PIXHOFFSET;
  int HCYCLE;
  int HOFFSET;
  int HSYNC0;
  int HSYNC1;
  int VCYCLE;
  int VOFFSET;
  int VSYNC0;
  int VSYNC1;
  int PCLK;
  int SWIZZLE;
  int PCLK_POL;
  int HSIZE;
  int VSIZE;
  int CSPREAD;
  int DITHER;

  switch (display)
  {
  case DISPLAY_70_800x480_WG:
  case DISPLAY_70_800x480:
    DWIDTH = 800;
    DHEIGHT = 480;
    PIXVOFFSET = 0;
    PIXHOFFSET = 0;
    HCYCLE = 928;
    HOFFSET = 88;
    HSYNC0 = 0;
    HSYNC1 = 48;
    VCYCLE = 525;
    VOFFSET = 32;
    VSYNC0 = 0;
    VSYNC1 = 3;
    PCLK = 2;
    SWIZZLE = 0;
    PCLK_POL = 1;
    HSIZE = 800;
    VSIZE = 480;
    CSPREAD = 0;
    DITHER = 1;
    break;
  case DISPLAY_50_800x480:
    DWIDTH = 800;
    DHEIGHT = 480;
    PIXVOFFSET = 0;
    PIXHOFFSET = 0;
    HCYCLE = 928;
    HOFFSET = 88;
    HSYNC0 = 0;
    HSYNC1 = 48;
    VCYCLE = 525;
    VOFFSET = 32;
    VSYNC0 = 0;
    VSYNC1 = 3;
    PCLK = 2;
    SWIZZLE = 0;
    PCLK_POL = 1;
    HSIZE = 800;
    VSIZE = 480;
    CSPREAD = 0;
    DITHER = 1;
    break;
  case DISPLAY_43_480x272:
    DWIDTH = 480;
    DHEIGHT = 272;
    PIXVOFFSET = 0;
    PIXHOFFSET = 0;
    HCYCLE = 548;
    HOFFSET = 43;
    HSYNC0 = 0;
    HSYNC1 = 41;
    VCYCLE = 292;
    VOFFSET = 12;
    VSYNC0 = 0;
    VSYNC1 = 10;
    PCLK = 5;
    SWIZZLE = 0;
    PCLK_POL = 1;
    HSIZE = 480;
    VSIZE = 272;
    CSPREAD = 1;
    DITHER = 1;
    break;
  case DISPLAY_43_800x480:
    DWIDTH = 800;
    DHEIGHT = 480;
    PIXVOFFSET = 0;
    PIXHOFFSET = 0;
    HCYCLE = 977;
    HOFFSET = 176;
    HSYNC0 = 40;
    HSYNC1 = 88;
    VCYCLE = 529;
    VOFFSET = 48;
    VSYNC0 = 13;
    VSYNC1 = 16;
    PCLK = 2;
    SWIZZLE = 0;
    PCLK_POL = 1;
    HSIZE = 800;
    VSIZE = 480;
    CSPREAD = 0;
    DITHER = 1;
    break;
  case DISPLAY_39_480x128:
    DWIDTH = 480;
    DHEIGHT = 128;
    PIXVOFFSET = 126;
    PIXHOFFSET = 0;
    HCYCLE = 552;
    HOFFSET = 71;
    HSYNC0 = 28;
    HSYNC1 = 44;
    VCYCLE = 308;
    VOFFSET = 35;
    VSYNC0 = 8;
    VSYNC1 = 11;
    PCLK = 6; //
    SWIZZLE = 0;
    PCLK_POL = 1;
    HSIZE = 480;
    VSIZE = 272;
    CSPREAD = 0;
    DITHER = 1;
    break;
  case DISPLAY_38_480x116:
    DWIDTH = 480;
    DHEIGHT = 116;
    PIXVOFFSET = 156;
    PIXHOFFSET = 0;
    HCYCLE = 527;
    HOFFSET = 46;
    HSYNC0 = 1;
    HSYNC1 = 3;
    VCYCLE = 291;
    VOFFSET = 18;
    VSYNC0 = 4;
    VSYNC1 = 6;
    PCLK = 5;
    SWIZZLE = 0;
    PCLK_POL = 1;
    HSIZE = 480;
    VSIZE = 272;
    CSPREAD = 1;
    DITHER = 1;
    break;
  case DISPLAY_35_320x240:
    DWIDTH = 320;
    DHEIGHT = 240;
    PIXVOFFSET = 0;
    PIXHOFFSET = 0;
    HCYCLE = 408;
    HOFFSET = 68;
    HSYNC0 = 0;
    HSYNC1 = 10;
    VCYCLE = 262;
    VOFFSET = 18;
    VSYNC0 = 0;
    VSYNC1 = 2;
    PCLK = 8;
    SWIZZLE = 0;
    PCLK_POL = 0;
    HSIZE = 320;
    VSIZE = 240;
    CSPREAD = 1;
    DITHER = 1;
    break;
  case DISPLAY_29_320x102:
    DWIDTH = 320;
    DHEIGHT = 105;
    PIXVOFFSET = 135;
    PIXHOFFSET = 0;
    HCYCLE = 429;
    HOFFSET = 108;
    HSYNC0 = 20;
    HSYNC1 = 40;
    VCYCLE = 283;
    VOFFSET = 42;
    VSYNC0 = 4;
    VSYNC1 = 24;
    PCLK = 9; //
    SWIZZLE = 0;
    PCLK_POL = 1;
    HSIZE = 320;
    VSIZE = 240;
    CSPREAD = 0;
    DITHER = 1;
    break;
  case DISPLAY_40_720x720:
    DWIDTH = 720;
    DHEIGHT = 720;
    PIXVOFFSET = 0;
    PIXHOFFSET = 0;
    HCYCLE = 812;
    HOFFSET = 91;
    HSYNC0 = 46;
    HSYNC1 = 48;
    VCYCLE = 756;
    VOFFSET = 35;
    VSYNC0 = 16;
    VSYNC1 = 18;
    PCLK = 2;
    SWIZZLE = 0;
    PCLK_POL = 1;
    HSIZE = 720;
    VSIZE = 720;
    CSPREAD = 0;
    DITHER = 0;
    break;
  case DISPLAY_101_1280x800:
    DWIDTH = 1280;
    DHEIGHT = 800;
    PIXVOFFSET = 0;
    PIXHOFFSET = 0;
    HCYCLE = 1440;
    HOFFSET = 158;
    HSYNC0 = 78;
    HSYNC1 = 80;
    VCYCLE = 823;
    VOFFSET = 22;
    VSYNC0 = 11;
    VSYNC1 = 12;
    PCLK = 1;
    SWIZZLE = 0;
    PCLK_POL = 0;
    HSIZE = 1280;
    VSIZE = 800;
    CSPREAD = 0;
    DITHER = 1;
    break;
  case DISPLAY_70_1024x600_WG:
  case DISPLAY_70_1024x600:
    DWIDTH = 1024;
    DHEIGHT = 600;
    PIXVOFFSET = 0;
    PIXHOFFSET = 0;
    HCYCLE = 1344;
    HOFFSET = 319;
    HSYNC0 = 12;
    HSYNC1 = 230;
    VCYCLE = 635;
    VOFFSET = 34;
    VSYNC0 = 12;
    VSYNC1 = 22;
    PCLK = 1;
    SWIZZLE = 0;
    PCLK_POL = 1;
    HSIZE = 1024;
    VSIZE = 600;
    CSPREAD = 0;
    DITHER = 1;
    break;
  case DISPLAY_24_320x240:
    DWIDTH = 240;
    DHEIGHT = 320;
    PIXVOFFSET = 0;
    PIXHOFFSET = 0;
    HCYCLE = 298;
    HOFFSET = 57;
    HSYNC0 = 38;
    HSYNC1 = 48;
    VCYCLE = 336;
    VOFFSET = 15;
    VSYNC0 = 8;
    VSYNC1 = 8;
    PCLK = 6;
    SWIZZLE = 0;
    PCLK_POL = 0;
    HSIZE = 240;
    VSIZE = 320;
    CSPREAD = 1;
    DITHER = 1;
    break;
  case DISPLAY_52_480x128:
    DWIDTH = 480;
    DHEIGHT = 128;
    PIXVOFFSET = 68;
    PIXHOFFSET = 0;
    HCYCLE = 536;
    HOFFSET = 55;
    HSYNC0 = 8;
    HSYNC1 = 12;
    VCYCLE = 297;
    VOFFSET = 24;
    VSYNC0 = 8;
    VSYNC1 = 12;
    PCLK = 6;
    SWIZZLE = 0;
    PCLK_POL = 1;
    HSIZE = 480;
    VSIZE = 272;
    CSPREAD = 0;
    DITHER = 1;
    break;
  case DISPLAY_101_1024x600_ILI:
  case DISPLAY_101_1024x600_GiX:
    DWIDTH = 1024;
    DHEIGHT = 600;
    PIXVOFFSET = 0;
    PIXHOFFSET = 0;
    HCYCLE = 1353;
    HOFFSET = 328;
    HSYNC0 = 160;
    HSYNC1 = 168;
    VCYCLE = 644;
    VOFFSET = 43;
    VSYNC0 = 12;
    VSYNC1 = 20;
    PCLK = 1;
    SWIZZLE = 0;
    PCLK_POL = 1;
    HSIZE = 1024;
    VSIZE = 600;
    CSPREAD = 0;
    DITHER = 1;
    break;
  default:
    printf("Unknown display type\n");
    return 0;
    break;
  }
  Width = DWIDTH;
  Height = DHEIGHT;
  HOffset = PIXHOFFSET;
  VOffset = PIXVOFFSET;
  Touch = touch;
  if (!Eve_Reset()) // Hard reset of the EVE chip
    return 0;

  // Wakeup EVE
  if (board >= BOARD_EVE3)
  {
    HostCommand(HCMD_CLKEXT);
  }
  HostCommand(HCMD_ACTIVE);
  HAL_Delay(300);

  for (int loop = 0; loop < 50; loop++)
  {
    Ready = Cmd_READ_REG_ID();
    if (Ready)
      break;
    HAL_Delay(5);
  }
  if (!Ready)
    return 1; // bridge detected but no eve found

  for (int loop = 0; loop < 50; loop++)
  {
    Ready = rd16(REG_CPU_RESET);
    if (Ready)
      break;
    HAL_Delay(5);
  }
  if (!Ready)
    return 1; // bridge detected but no eve found

  //  Log("EVE now ACTIVE\n");         //

  Ready = rd32(REG_CHIP_ID);
  uint16_t ValH = Ready >> 16;
  uint16_t ValL = Ready & 0xFFFF;
  Log("Chip ID = 0x%04x%04x\n", ValH, ValL);

  if (display == DISPLAY_101_1280x800)
  {
    wr32(REG_FREQUENCY + RAM_REG, 80000000); // Configure the system clock to 80MHz
  }
  else
  {
    wr32(REG_FREQUENCY + RAM_REG, 60000000); // Configure the system clock to 60MHz
  }
  // Before we go any further with EVE, it is a good idea to check to see if the EVE is wigging out
  // about something that happened before the last reset.  If EVE has just done a power cycle, this
  // would be unnecessary.
  if (rd16(REG_CMD_READ + RAM_REG) == 0xFFF)
  {
    // EVE is unhappy - needs a paddling.
    uint32_t Patch_Add = rd32(REG_COPRO_PATCH_PTR + RAM_REG);
    wr8(REG_CPU_RESET + RAM_REG, 1);
    wr16(REG_CMD_READ + RAM_REG, 0);
    wr16(REG_CMD_WRITE + RAM_REG, 0);
    wr16(REG_CMD_DL + RAM_REG, 0);
    wr8(REG_CPU_RESET + RAM_REG, 0);
    wr32(REG_COPRO_PATCH_PTR + RAM_REG, Patch_Add);
  }

  // Turn off screen output during startup
  wr16(REG_GPIOX + RAM_REG,
       rd16(REG_GPIOX + RAM_REG) &
           ~(1 << 15));       // Set REG_GPIOX bit 15 to 0 to turn off the LCD DISP signal
  wr8(REG_PCLK + RAM_REG, 0); // Pixel Clock Output disable

  if (display == DISPLAY_24_320x240)
  {
    MO_ST7789V_init();
  }

  // Load parameters of the physical screen to the EVE
  // All of these registers are 32 bits, but most bits are reserved, so only write what is actually
  // used
  wr16(REG_HCYCLE + RAM_REG, HCYCLE);    // Set H_Cycle to 548
  wr16(REG_HOFFSET + RAM_REG, HOFFSET);  // Set H_Offset to 43
  wr16(REG_HSYNC0 + RAM_REG, HSYNC0);    // Set H_SYNC_0 to 0
  wr16(REG_HSYNC1 + RAM_REG, HSYNC1);    // Set H_SYNC_1 to 41
  wr16(REG_VCYCLE + RAM_REG, VCYCLE);    // Set V_Cycle to 292
  wr16(REG_VOFFSET + RAM_REG, VOFFSET);  // Set V_OFFSET to 12
  wr16(REG_VSYNC0 + RAM_REG, VSYNC0);    // Set V_SYNC_0 to 0
  wr16(REG_VSYNC1 + RAM_REG, VSYNC1);    // Set V_SYNC_1 to 10
  wr8(REG_SWIZZLE + RAM_REG, SWIZZLE);   // Set SWIZZLE to 0
  wr8(REG_PCLK_POL + RAM_REG, PCLK_POL); // Set PCLK_POL to 1
  wr16(REG_HSIZE + RAM_REG, HSIZE);      // Set H_SIZE to 480
  wr16(REG_VSIZE + RAM_REG, VSIZE);      // Set V_SIZE to 272
  wr8(REG_CSPREAD + RAM_REG, CSPREAD); // Set CSPREAD to 1    (32 bit register - write only 8 bits)
  wr8(REG_DITHER + RAM_REG, DITHER);   // Set DITHER to 1     (32 bit register - write only 8 bits)

  /* Reset the touch engine, since it has sometimes issues starting up. */
  wr32(RAM_REG + REG_CPU_RESET, 1 << 1);
  HAL_Delay(10);
  wr32(RAM_REG + REG_CPU_RESET, 0);
  HAL_Delay(10);
  // Configure touch & audio
  if (touch == TOUCH_TPR)
  {
    wr16(REG_TOUCH_CONFIG + RAM_REG, 0x8381);
  }
  else if (touch == TOUCH_TPC)
  {
    if (display == DISPLAY_40_720x720)
      wr16(REG_TOUCH_CONFIG + RAM_REG, 0x480); // FT6336U touch controller
    else
      wr16(REG_TOUCH_CONFIG + RAM_REG, 0x5d0);
    if (board == BOARD_EVE2)
    {
      Cap_Touch_Upload();
    }
    if (display == DISPLAY_70_1024x600_WG || display == DISPLAY_70_800x480_WG ||
        display == DISPLAY_101_1024x600_ILI)
    {
      UploadTouchFirmware(Touch_il, sizeof(Touch_il));
      Calibrate_Fixed(Display_Width(), Display_Height(), 16384, 16384);
    }
    if (display == DISPLAY_52_480x128)
    {
      UploadTouchFirmware(Touch_cyt, sizeof(Touch_cyt));
    }
  }

  wr16(REG_TOUCH_RZTHRESH + RAM_REG, 1200); // Set touch resistance threshold
  wr8(REG_TOUCH_MODE + RAM_REG, 0x02);      // Set touch on: continous - this is default
  wr8(REG_TOUCH_ADC_MODE + RAM_REG, 0x01);  // Set ADC mode: differential - this is default
  wr8(REG_TOUCH_OVERSAMPLE + RAM_REG, 15);  // Set touch oversampling to max

  // wr16(REG_GPIOX_DIR + RAM_REG, 0x8000 | (1<<3));   // Set Disp GPIO Direction
  // wr16(REG_GPIOX + RAM_REG, 0x8000 | (1<<3));       // Enable Disp (if used)

  wr16(REG_GPIOX_DIR + RAM_REG, 0xffff); // Make GPIOs output
  if (display == DISPLAY_101_1280x800)
  {
    wr16(REG_GPIOX + RAM_REG,
         0x80f7); // Motor (GPIO 3, active high) is off, speaker (GPIO 2) is on
  }
  else
  {
    wr16(REG_GPIOX + RAM_REG, 0x80ff); // Motor (GPIO 3, active low) is off, speaker (GPIO 2) is on
  }

  wr16(REG_PWM_HZ + RAM_REG, 0x00FA); // Backlight PWM frequency
  wr8(REG_PWM_DUTY + RAM_REG, 128);   // Backlight PWM duty (on)

  // write first display list (which is a clear and blank screen)
  wr32(RAM_DL + 0, CLEAR_COLOR_RGB(0, 0, 0));
  wr32(RAM_DL + 4, CLEAR(1, 1, 1));
  wr32(RAM_DL + 8, DISPLAY());
  wr8(REG_DLSWAP + RAM_REG, DLSWAP_FRAME); // Swap display lists
  wr8(REG_PCLK + RAM_REG, PCLK);           // After this display is visible on the TFT
  return Ready;
}

// Reset EVE chip via the hardware PDN line
int Eve_Reset(void)
{
  FifoWriteLocation = 0;
  return HAL_Eve_Reset_HW();
}

// Upload Goodix Calibration file, ex GT911
void Cap_Touch_Upload(void)
{
  // This makes the Arduino uno run out of space so sadly
  // we cannot support this.
#if !defined(__AVR__)
  //---Goodix911 Configuration from AN336
  // Load the TOUCH_DATA_U8 or TOUCH_DATA_U32 array from file “touch_cap_811.h” via the FT81x
  // command buffer RAM_CMD
  uint8_t CTOUCH_CONFIG_DATA_G911[] = {
      26,  255, 255, 255, 32,  32,  48,  0,   4,   0,   0,   0,   2,   0,   0,   0,   34,  255,
      255, 255, 0,   176, 48,  0,   120, 218, 237, 84,  221, 111, 84,  69,  20,  63,  51,  179,
      93,  160, 148, 101, 111, 76,  5,   44,  141, 123, 111, 161, 11,  219, 154, 16,  9,   16,
      17,  229, 156, 75,  26,  11,  13,  21,  227, 3,   16,  252, 184, 179, 45,  219, 143, 45,
      41,  125, 144, 72,  67,  100, 150, 71,  189, 113, 18,  36,  17,  165, 100, 165, 198, 16,
      32,  17,  149, 196, 240, 128, 161, 16,  164, 38,  54,  240, 0,   209, 72,  130, 15,  38,
      125, 48,  66,  82,  30,  76,  19,  31,  172, 103, 46,  139, 24,  255, 4,   227, 157, 204,
      156, 51,  115, 102, 206, 231, 239, 220, 5,   170, 94,  129, 137, 75,  194, 216, 98,  94,
      103, 117, 115, 121, 76,  131, 177, 125, 89,  125, 82,  123, 60,  243, 58,  142, 242, 204,
      185, 243, 188, 118, 156, 227, 155, 203, 238, 238, 195, 251, 205, 229, 71,  92,  28,  169,
      190, 184, 84,  143, 113, 137, 53,  244, 103, 181, 237, 87,  253, 113, 137, 233, 48,  12,
      198, 165, 181, 104, 139, 25,  84,  253, 155, 114, 74,  191, 0,   54,  138, 163, 12,  62,
      131, 207, 129, 23,  217, 34,  91,  31,  128, 65,  246, 163, 175, 213, 8,   147, 213, 107,
      35,  203, 94,  108, 3,   111, 40,  171, 83,  24,  15,  165, 177, 222, 116, 97,  23,  188,
      140, 206, 150, 42,  102, 181, 87,  78,  86,  182, 170, 134, 215, 241, 121, 26,  243, 252,
      2,   76,  115, 217, 139, 222, 206, 173, 136, 132, 81,  61,  35,  185, 39,  113, 23,  46,
      199, 76,  178, 54,  151, 183, 224, 0,   40,  189, 28,  149, 182, 58,  131, 79,  152, 30,
      76,  34,  98,  234, 162, 216, 133, 141, 102, 39,  170, 40,  192, 101, 53,  201, 146, 191,
      37,  77,  44,  177, 209, 74,  211, 5,   206, 187, 5,   6,   216, 47,  53,  96,  123, 22,
      50,  103, 251, 192, 84,  17,  74,  227, 185, 56,  106, 51,  91,  161, 96,  182, 163, 48,
      171, 141, 139, 65,  152, 66,  66,  11,  102, 43,  158, 75,  36,  80,  147, 184, 147, 139,
      112, 17,  235, 216, 103, 111, 239, 245, 92,  10,  175, 194, 40,  44,  58,  125, 5,   59,
      112, 50,  103, 245, 4,   78,  192, 5,   156, 194, 51,  60,  191, 134, 75,  110, 173, 237,
      46,  192, 121, 156, 192, 115, 184, 218, 120, 67,  63,  115, 46,  11,  102, 10,  97,  232,
      50,  235, 114, 182, 148, 118, 178, 41,  188, 12,  135, 77,  202, 124, 12,  96,  238, 35,
      161, 234, 189, 129, 23,  249, 212, 139, 230, 25,  53,  48,  205, 52,  93,  163, 117, 53,
      154, 170, 81,  85,  163, 178, 70,  69,  66,  167, 241, 14,  46,  241, 1,   226, 136, 152,
      179, 197, 59,  184, 148, 254, 49,  132, 48,  15,  176, 137, 192, 76,  131, 196, 105, 104,
      162, 86,  81,  160, 165, 255, 26,  173, 162, 137, 86,  145, 210, 183, 192, 55,  175, 194,
      211, 60,  91,  120, 230, 184, 174, 27,  41,  131, 155, 40,  224, 29,  87,  179, 232, 16,
      55,  55,  7,   165, 147, 81,  23,  165, 49,  101, 54,  224, 75,  180, 81,  108, 18,  29,
      226, 69,  225, 110, 175, 224, 42,  212, 25,  47,  130, 193, 110, 234, 192, 215, 252, 56,
      74,  162, 24,  46,  251, 174, 54,  106, 68,  245, 14,  9,   155, 160, 22,  120, 207, 104,
      240, 29,  90,  178, 140, 28,  24,  220, 47,  166, 112, 61,  251, 208, 192, 111, 56,  239,
      238, 93,  255, 251, 62,  99,  32,  193, 75,  61,  190, 235, 123, 229, 110, 218, 194, 85,
      79,  225, 59,  98,  20,  238, 227, 235, 220, 11,  221, 149, 25,  180, 116, 194, 159, 111,
      96,  192, 24,  213, 59,  139, 179, 156, 215, 69,  230, 19,  24,  35,  135, 117, 206, 171,
      206, 162, 67,  129, 234, 61,  235, 11,  104, 103, 84,  64,  223, 167, 254, 40,  163, 101,
      92,  84,  43,  150, 46,  249, 219, 205, 7,   116, 11,  91,  104, 61,  57,  75,  223, 8,
      48,  25,  28,  119, 252, 222, 113, 49,  86,  249, 74,  180, 211, 156, 181, 61,  215, 168,
      157, 7,   251, 199, 150, 242, 250, 91,  58,  132, 94,  121, 7,   53,  151, 139, 98,  6,
      165, 153, 69,  214, 32,  110, 211, 100, 101, 31,  89,  45,  81,  98,  23,  205, 205, 197,
      209, 109, 186, 198, 35,  141, 191, 249, 25,  60,  132, 223, 153, 251, 98,  20,  239, 146,
      139, 20,  217, 250, 41,  250, 137, 58,  177, 90,  57,  79,  51,  108, 233, 20,  253, 194,
      187, 49,  222, 205, 114, 141, 96,  48,  175, 219, 107, 54,  111, 138, 22,  154, 103, 108,
      79,  58,  252, 179, 178, 79,  164, 195, 2,   153, 36,  39,  170, 199, 201, 167, 197, 85,
      106, 8,   59,  177, 81,  46,  56,  2,   230, 75,  114, 17,  55,  112, 188, 65,  208, 137,
      77,  114, 10,  115, 55,  58,  208, 197, 173, 122, 87,  6,   140, 110, 42,  208, 124, 163,
      70,  108, 241, 104, 18,  245, 98,  214, 187, 134, 53,  42,  221, 22,  182, 133, 211, 116,
      148, 177, 194, 209, 192, 85,  90,  199, 58,  55,  203, 2,   229, 19,  137, 187, 161, 228,
      154, 112, 203, 145, 125, 244, 188, 220, 118, 228, 41,  201, 181, 41,  195, 144, 215, 183,
      51,  80,  250, 21,  217, 16,  217, 200, 235, 109, 227, 188, 122, 218, 142, 60,  170, 224,
      112, 240, 184, 130, 229, 224, 113, 5,   223, 148, 163, 80,  165, 183, 130, 187, 132, 116,
      64,  238, 161, 85,  220, 115, 139, 205, 98,  227, 244, 29,  102, 125, 7,   37,  243, 123,
      223, 11,  26,  92,  63,  243, 116, 61,  191, 138, 123, 244, 160, 84,  186, 74,  31,  5,
      174, 247, 119, 135, 199, 248, 253, 135, 242, 97,  102, 145, 190, 144, 14,  85,  238, 221,
      231, 193, 158, 48,  205, 25,  120, 248, 15,  220, 29,  158, 9,   70,  185, 30,  103, 229,
      33,  254, 23,  237, 160, 172, 62,  193, 90,  222, 224, 232, 14,  200, 56,  90,  104, 142,
      227, 120, 110, 6,   21,  211, 203, 65,  150, 99,  151, 220, 247, 87,  164, 50,  159, 49,
      239, 234, 58,  142, 0,   109, 108, 123, 18,  79,  227, 36,  100, 248, 222, 205, 96,  127,
      120, 26,  171, 228, 69,  63,  36,  17,  252, 200, 17,  116, 242, 187, 227, 88,  143, 247,
      2,   75,  191, 6,   130, 59,  188, 11,  55,  240, 31,  243, 122, 152, 226, 183, 207, 154,
      73,  188, 39,  219, 43,  105, 222, 87,  41,  143, 141, 140, 175, 73,  112, 184, 252, 61,
      184, 16,  90,  250, 35,  168, 82,  119, 176, 57,  116, 94,  200, 150, 22,  190, 179, 44,
      104, 12,  235, 84,  149, 102, 252, 89,  154, 193, 99,  228, 106, 242, 125, 248, 64,  194,
      255, 223, 127, 242, 83,  11,  255, 2,   70,  214, 226, 128, 0,   0,   26,  255, 255, 255,
      20,  33,  48,  0,   4,   0,   0,   0,   15,  0,   0,   0,   26,  255, 255, 255, 32,  32,
      48,  0,   4,   0,   0,   0,   0,   0,   0,   0};
  CoProWrCmdBuf(CTOUCH_CONFIG_DATA_G911, sizeof(CTOUCH_CONFIG_DATA_G911));
  // Execute the commands till completion
  UpdateFIFO();
  Wait4CoProFIFOEmpty();
  // Hold the touch engine in reset(write REG_CPURESET = 2)
  wr8(REG_CPU_RESET + RAM_REG, 2);
  // Set GPIO3 output LOW
  wr8(REG_GPIOX_DIR + RAM_REG, (rd8(RAM_REG + REG_GPIOX_DIR) | 0x08)); // Set Disp GPIO Direction
  wr8(REG_GPIOX + RAM_REG, (rd8(RAM_REG + REG_GPIOX) | 0xF7));         // Clear GPIO
  // Wait more than 100us
  HAL_Delay(1);
  // Write REG_CPURESET=0
  wr8(REG_CPU_RESET + RAM_REG, 0);
  // Wait more than 55ms
  HAL_Delay(100);
  // Set GPIO3 to input (floating)
  wr8(REG_GPIOX_DIR + RAM_REG, (rd8(RAM_REG + REG_GPIOX_DIR) & 0xF7)); // Set Disp GPIO Direction
#endif
}

// *** Host Command - FT81X Embedded Video Engine Datasheet - 4.1.5
// ********************************************** Host Command is a function for changing hardware
// related parameters of the Eve chip.  The name is confusing. These are related to power modes and
// the like.  All defined parameters have HCMD_ prefix
void HostCommand(uint8_t HCMD)
{
  //  Log("Inside HostCommand\n");

  HAL_SPI_Enable();

  /*  HAL_SPI_Write(HCMD | 0x40); // In case the manual is making you believe that you just found
   * the bug you were looking for - no. */
  HAL_SPI_Write(HCMD);
  HAL_SPI_Write(0x00); // This second byte is set to 0 but if there is need for fancy, never used
                       // setups, then rewrite.
  HAL_SPI_Write(0x00);

  HAL_SPI_Disable();
}

// *** EVE API Reference Definitions
// ***************************************************************************** FT81X Embedded
// Video Engine Datasheet 1.3 - Section 4.1.4, page 16 These are all functions related to writing /
// reading data of various lengths with a memory address of 32 bits
// ***************************************************************************************************************
void wr32(uint32_t address, uint32_t parameter)
{
  HAL_SPI_Enable();
  uint8_t buffer[16];
  int idx = 0;

  buffer[idx++] =
      ((address >> 16) | 0x80); // RAM_REG = 0x302000 and high bit is set - result always 0xB0
  buffer[idx++] = (uint8_t)(address >> 8); // Next byte of the register address
  buffer[idx++] =
      (uint8_t)address; // Low byte of register address - usually just the 1 byte offset

  buffer[idx++] = (uint8_t)(parameter & 0xff); // Little endian (yes, it is most significant bit
                                               // first and least significant byte first)
  buffer[idx++] = (uint8_t)((parameter >> 8) & 0xff);
  buffer[idx++] = (uint8_t)((parameter >> 16) & 0xff);
  buffer[idx++] = (uint8_t)((parameter >> 24) & 0xff);
  HAL_SPI_WriteBuffer(buffer, idx);

  HAL_SPI_Disable();
}

void wr16(uint32_t address, uint16_t parameter)
{
  HAL_SPI_Enable();

  HAL_SPI_Write((uint8_t)((address >> 16) |
                          0x80)); // RAM_REG = 0x302000 and high bit is set - result always 0xB0
  HAL_SPI_Write((uint8_t)(address >> 8)); // Next byte of the register address
  HAL_SPI_Write((uint8_t)address); // Low byte of register address - usually just the 1 byte offset

  HAL_SPI_Write((uint8_t)(parameter & 0xff)); // Little endian (yes, it is most significant bit
                                              // first and least significant byte first)
  HAL_SPI_Write((uint8_t)(parameter >> 8));

  HAL_SPI_Disable();
}

void wr8(uint32_t address, uint8_t parameter)
{
  HAL_SPI_Enable();

  HAL_SPI_Write((uint8_t)((address >> 16) |
                          0x80)); // RAM_REG = 0x302000 and high bit is set - result always 0xB0
  HAL_SPI_Write((uint8_t)(address >> 8)); // Next byte of the register address
  HAL_SPI_Write(
      (uint8_t)(address)); // Low byte of register address - usually just the 1 byte offset

  HAL_SPI_Write(parameter);

  HAL_SPI_Disable();
}

uint32_t rd32(uint32_t address)
{
  uint8_t buf[4];
  int idx = 0;
  uint32_t Data32;

  HAL_SPI_Enable();

  buf[idx++] = (address >> 16) & 0x3F;
  buf[idx++] = (address >> 8) & 0xff;
  buf[idx++] = address & 0xff;
  HAL_SPI_WriteBuffer(buf, idx);
  HAL_SPI_ReadBuffer(buf, 4);

  HAL_SPI_Disable();

  Data32 = buf[0] + ((uint32_t)buf[1] << 8) + ((uint32_t)buf[2] << 16) + ((uint32_t)buf[3] << 24);
  return (Data32);
}
uint16_t rd16(uint32_t address)
{
  uint8_t buf[2] = {0, 0};

  HAL_SPI_Enable();

  HAL_SPI_Write((address >> 16) & 0x3F);
  HAL_SPI_Write((address >> 8) & 0xff);
  HAL_SPI_Write(address & 0xff);

  HAL_SPI_ReadBuffer(buf, 2);

  HAL_SPI_Disable();

  uint16_t Data16 = buf[0] + ((uint16_t)buf[1] << 8);
  return (Data16);
}

uint8_t rd8(uint32_t address)
{
  uint8_t buf[1];

  HAL_SPI_Enable();

  HAL_SPI_Write((address >> 16) & 0x3F);
  HAL_SPI_Write((address >> 8) & 0xff);
  HAL_SPI_Write(address & 0xff);

  HAL_SPI_ReadBuffer(buf, 1);

  HAL_SPI_Disable();

  return (buf[0]);
}

void rdN(uint32_t address, uint8_t *buffer, uint32_t size)
{

  HAL_SPI_Enable();

  HAL_SPI_Write((address >> 16) & 0x3F);
  HAL_SPI_Write((address >> 8) & 0xff);
  HAL_SPI_Write(address & 0xff);

  HAL_SPI_ReadBuffer(buffer, size);

  HAL_SPI_Disable();
}

// *** Send_Cmd() - this is like cmd() in (some) EVE docs - sends 32 bits but does not update the
// write pointer *** FT81x Series Programmers Guide Section 5.1.1 - Circular Buffer (AKA "the FIFO"
// and "Command buffer" and "Coprocessor") Don't miss section 5.3 - Interaction with RAM_DL
void Send_CMD(uint32_t data)
{
  wr32(FifoWriteLocation + RAM_CMD,
       data); // Write the command at the globally tracked "write pointer" for the FIFO

  FifoWriteLocation +=
      FT_CMD_SIZE; // Increment the Write Address by the size of a command - which we just sent
  FifoWriteLocation %= FT_CMD_FIFO_SIZE; // Wrap the address to the FIFO space
}

// UpdateFIFO - Cause the coprocessor to realize that it has work to do in the form of a
// differential between the read pointer and write pointer.  The coprocessor (FIFO or "Command
// buffer") does nothing until you tell it that the write position in the FIFO RAM has changed
void UpdateFIFO(void)
{
  wr16(REG_CMD_WRITE + RAM_REG,
       FifoWriteLocation); // We manually update the write position pointer
}

// Read the specific ID register and return TRUE if it is the expected 0x7C otherwise.
uint8_t Cmd_READ_REG_ID(void)
{
  uint8_t readData[2];

  HAL_SPI_Enable();
  HAL_SPI_Write(0x30); // Base address RAM_REG = 0x302000
  HAL_SPI_Write(0x20);
  HAL_SPI_Write(REG_ID);           // REG_ID offset = 0x00
  HAL_SPI_ReadBuffer(readData, 1); // There was a dummy read of the first byte in there
  HAL_SPI_Disable();

  if (readData[0] == 0x7C) // FT81x Datasheet section 5.1, Table 5-2. Return value always 0x7C
  {
    //    Log("\nGood ID: 0x%02x\n", readData[0]);
    return 1;
  }
  else
  {
    //    Log("0x%02x ", readData[0]);
    return 0;
  }
}

// **************************************** Coprocessor/GPU/FIFO/Command buffer Command Functions
// *************** These are discussed in FT81x Series Programmers Guide, starting around
// section 5.10 While display list commands can be sent to the coprocessor, these listed commands
// are specific to it.  They are mostly widgets like graphs, but also touch related functions like
// cmd_track() and memory operations. Essentially, these commands set up parameters for CoPro
// functions which expand "macros" using those parameters to then write a series of commands into
// the Display List to create all the primitives which make that widget.
// ***************************************************************************************************************

// ******************** Screen Object Creation Coprocessor Command Functions
// ******************************

void Cmd_Progress(
    uint16_t x, uint16_t y, uint16_t w, uint16_t h, uint16_t options, uint16_t val, uint16_t range)
{
  Send_CMD(CMD_PROGRESS);
  Send_CMD(((uint32_t)y << 16) | x);
  Send_CMD(((uint32_t)h << 16) | w);
  Send_CMD(((uint32_t)val << 16) | options);
  Send_CMD((uint32_t)range);
}

// *** Draw Slider - FT81x Series Programmers Guide Section 5.38
// *************************************************
void Cmd_Slider(
    uint16_t x, uint16_t y, uint16_t w, uint16_t h, uint16_t options, uint16_t val, uint16_t range)
{
  Send_CMD(CMD_SLIDER);
  Send_CMD(((uint32_t)y << 16) | x);
  Send_CMD(((uint32_t)h << 16) | w);
  Send_CMD(((uint32_t)val << 16) | options);
  Send_CMD((uint32_t)range);
}

// *** Draw Spinner - FT81x Series Programmers Guide Section 5.54
// *************************************************
void Cmd_Spinner(uint16_t x, uint16_t y, uint16_t style, uint16_t scale)
{
  Send_CMD(CMD_SPINNER);
  Send_CMD(((uint32_t)y << 16) | x);
  Send_CMD(((uint32_t)scale << 16) | style);
}

// *** Draw Gauge - FT81x Series Programmers Guide Section 5.33
// **************************************************
void Cmd_Gauge(uint16_t x,
               uint16_t y,
               uint16_t r,
               uint16_t options,
               uint16_t major,
               uint16_t minor,
               uint16_t val,
               uint16_t range)
{
  Send_CMD(CMD_GAUGE);
  Send_CMD(((uint32_t)y << 16) | x);
  Send_CMD(((uint32_t)options << 16) | r);
  Send_CMD(((uint32_t)minor << 16) | major);
  Send_CMD(((uint32_t)range << 16) | val);
}

// *** Draw Dial - FT81x Series Programmers Guide Section 5.39
// ************************************************** This is much like a Gauge except for the
// helpful range parameter.  For some reason, all dials are 65535 around.
void Cmd_Dial(uint16_t x, uint16_t y, uint16_t r, uint16_t options, uint16_t val)
{
  Send_CMD(CMD_DIAL);
  Send_CMD(((uint32_t)y << 16) | x);
  Send_CMD(((uint32_t)options << 16) | r);
  Send_CMD((uint32_t)val);
}

// *** Make Track (for a slider) - FT81x Series Programmers Guide Section 5.62
// ************************************ Tag refers to the tag # previously assigned to the object
// that this track is tracking.
void Cmd_Track(uint16_t x, uint16_t y, uint16_t w, uint16_t h, uint16_t tag)
{
  Send_CMD(CMD_TRACK);
  Send_CMD(((uint32_t)y << 16) | x);
  Send_CMD(((uint32_t)h << 16) | w);
  Send_CMD((uint32_t)tag);
}

// *** Draw Number - FT81x Series Programmers Guide Section 5.43
// *************************************************
void Cmd_Number(uint16_t x, uint16_t y, uint16_t font, uint16_t options, uint32_t num)
{
  Send_CMD(CMD_NUMBER);
  Send_CMD(((uint32_t)y << 16) | x);
  Send_CMD(((uint32_t)options << 16) | font);
  Send_CMD(num);
}

// *** Draw Smooth Color Gradient - FT81x Series Programmers Guide Section 5.34
// **********************************
void Cmd_Gradient(uint16_t x0, uint16_t y0, uint32_t rgb0, uint16_t x1, uint16_t y1, uint32_t rgb1)
{
  Send_CMD(CMD_GRADIENT);
  Send_CMD(((uint32_t)y0 << 16) | x0);
  Send_CMD(rgb0);
  Send_CMD(((uint32_t)y1 << 16) | x1);
  Send_CMD(rgb1);
}

// *** Draw Button - FT81x Series Programmers Guide Section 5.28
// **************************************************
void Cmd_Button(uint16_t x,
                uint16_t y,
                uint16_t w,
                uint16_t h,
                uint16_t font,
                uint16_t options,
                const char *str)
{
  uint16_t DataPtr, LoopCount, StrPtr;

  uint16_t length = (uint16_t)strlen(str);
  if (!length)
    return;

  uint32_t *data = (uint32_t *)calloc((length / 4) + 1, sizeof(uint32_t));

  StrPtr = 0;
  for (DataPtr = 0; DataPtr < (length / 4); DataPtr++, StrPtr += 4)
    data[DataPtr] = (uint32_t)str[StrPtr + 3] << 24 | (uint32_t)str[StrPtr + 2] << 16 |
                    (uint32_t)str[StrPtr + 1] << 8 | (uint32_t)str[StrPtr];

  for (LoopCount = 0; LoopCount < (length % 4); LoopCount++, StrPtr++)
    data[DataPtr] |= (uint32_t)str[StrPtr] << (LoopCount * 8);

  Send_CMD(CMD_BUTTON);
  Send_CMD(((uint32_t)y << 16) |
           x); // Put two 16 bit values together into one 32 bit value - do it little endian
  Send_CMD(((uint32_t)h << 16) | w);
  Send_CMD(((uint32_t)options << 16) | font);

  for (LoopCount = 0; LoopCount <= length / 4; LoopCount++)
  {
    Send_CMD(data[LoopCount]);
  }

  free(data);
}

// *** Draw Text - FT81x Series Programmers Guide Section 5.41
// ***************************************************
void Cmd_Text(uint16_t x, uint16_t y, uint16_t font, uint16_t options, const char *str)
{
  uint16_t length = (uint16_t)strlen(str);
  if (!length)
    return;

  uint32_t *data = (uint32_t *)calloc((length / 4) + 1, sizeof(uint32_t));

  uint16_t StrPtr = 0;
  uint16_t DataPtr;

  for (DataPtr = 0; DataPtr < (length / 4); ++DataPtr, StrPtr += 4)
  {
    data[DataPtr] = (uint32_t)(uint8_t)str[StrPtr + 3] << 24 |
                    (uint32_t)(uint8_t)str[StrPtr + 2] << 16 |
                    (uint32_t)(uint8_t)str[StrPtr + 1] << 8 | (uint32_t)(uint8_t)str[StrPtr];
  }

  // Handle remaining bytes
  for (uint16_t LoopCount = 0; LoopCount < (length % 4); ++LoopCount, ++StrPtr)
  {
    data[DataPtr] |= (uint32_t)(uint8_t)str[StrPtr] << (LoopCount * 8);
  }

  // Send command
  Send_CMD(CMD_TEXT);
  Send_CMD(((uint32_t)y << 16) | x);
  Send_CMD(((uint32_t)options << 16) | font);

  // Send text data
  for (uint16_t LoopCount = 0; LoopCount <= length / 4; LoopCount++)
  {
    Send_CMD(data[LoopCount]);
  }

  free(data);
}

// ******************** Miscellaneous Operation Coprocessor Command Functions
// ******************************
void Cmd_SetFont2(uint32_t handle, uint32_t addr, uint32_t firstChar)
{
  Send_CMD(CMD_SETFONT2);
  Send_CMD(handle);
  Send_CMD(addr);
  Send_CMD(firstChar);
}
// *** Cmd_SetBitmap - generate DL commands for bitmap parms - FT81x Series Programmers Guide
// Section 5.65 *******
void Cmd_SetBitmap(uint32_t addr, uint16_t fmt, uint16_t width, uint16_t height)
{
  Send_CMD(CMD_SETBITMAP);
  Send_CMD(addr);
  uint32_t val = width;
  val = val << 16;
  val = val | fmt;
  Send_CMD(val);
  Send_CMD((uint32_t)height);
}

// *** Cmd_Memcpy - background copy a block of data - FT81x Series Programmers Guide Section 5.27
// ****************
void Cmd_Memcpy(uint32_t dest, uint32_t src, uint32_t num)
{
  Send_CMD(CMD_MEMCPY);
  Send_CMD(dest);
  Send_CMD(src);
  Send_CMD(num);
}

// *** Cmd_GetPtr - Get the last used address from CoPro operation - FT81x Series Programmers Guide
// Section 5.47 *
void Cmd_GetPtr(void)
{
  Send_CMD(CMD_GETPTR);
  Send_CMD(0);
}

// *** Set Highlight Gradient Color - FT81x Series Programmers Guide Section 5.32
// ********************************
void Cmd_GradientColor(uint32_t c)
{
  Send_CMD(CMD_GRADCOLOR);
  Send_CMD(c);
}

// *** Set Foreground color - FT81x Series Programmers Guide Section 5.30
// ************************************************
void Cmd_FGcolor(uint32_t c)
{
  Send_CMD(CMD_FGCOLOR);
  Send_CMD(c);
}

// *** Set Background color - FT81x Series Programmers Guide Section 5.31
// ************************************************
void Cmd_BGcolor(uint32_t c)
{
  Send_CMD(CMD_BGCOLOR);
  Send_CMD(c);
}

// *** Translate Matrix - FT81x Series Programmers Guide Section 5.51
// ********************************************
void Cmd_Translate(uint32_t tx, uint32_t ty)
{
  Send_CMD(CMD_TRANSLATE);
  Send_CMD(tx);
  Send_CMD(ty);
}

// *** Rotate Matrix - FT81x Series Programmers Guide Section 5.50
// ***********************************************
void Cmd_Rotate(uint32_t a)
{
  Send_CMD(CMD_ROTATE);
  Send_CMD(a);
}

// *** Rotate Screen - FT81x Series Programmers Guide Section 5.53
// ***********************************************
void Cmd_SetRotate(uint32_t rotation)
{
  Send_CMD(CMD_SETROTATE);
  Send_CMD(rotation);
}

// *** Scale Matrix - FT81x Series Programmers Guide Section 5.49
// ************************************************
void Cmd_Scale(uint32_t sx, uint32_t sy)
{
  Send_CMD(CMD_SCALE);
  Send_CMD(sx);
  Send_CMD(sy);
}

// *** Flash Fast - FT81x Series Programmers Guide Section x.xx
// ************************************************
void Cmd_Flash_Fast(void)
{
  Send_CMD(CMD_FLASHFAST);
  Send_CMD(0);
}

// *** Calibrate Touch Digitizer - FT81x Series Programmers Guide Section 5.52
// ***********************************
// * This business about "result" in the manual really seems to be simply leftover cruft of no
// purpose - send zero
void Cmd_Calibrate(uint32_t result)
{
  Send_CMD(CMD_CALIBRATE);
  Send_CMD(result);
}

void calculate_touch_matrix(uint32_t displayX[3],
                            uint32_t displayY[3],
                            uint32_t touchX[3],
                            uint32_t touchY[3],
                            uint32_t out_TransMatrix[6])
{
  int32_t tmp, k;
  k = ((touchX[0] - touchX[2]) * (touchY[1] - touchY[2])) -
      ((touchX[1] - touchX[2]) * (touchY[0] - touchY[2]));

  tmp = (((displayX[0] - displayX[2]) * (touchY[1] - touchY[2])) -
         ((displayX[1] - displayX[2]) * (touchY[0] - touchY[2])));
  out_TransMatrix[0] = ((int64_t)tmp << 16) / k;

  tmp = (((touchX[0] - touchX[2]) * (displayX[1] - displayX[2])) -
         ((displayX[0] - displayX[2]) * (touchX[1] - touchX[2])));
  out_TransMatrix[1] = ((int64_t)tmp << 16) / k;

  tmp = ((touchY[0] * (((touchX[2] * displayX[1]) - (touchX[1] * displayX[2])))) +
         (touchY[1] * (((touchX[0] * displayX[2]) - (touchX[2] * displayX[0])))) +
         (touchY[2] * (((touchX[1] * displayX[0]) - (touchX[0] * displayX[1])))));
  out_TransMatrix[2] = ((int64_t)tmp << 16) / k;

  tmp = (((displayY[0] - displayY[2]) * (touchY[1] - touchY[2])) -
         ((displayY[1] - displayY[2]) * (touchY[0] - touchY[2])));
  out_TransMatrix[3] = ((int64_t)tmp << 16) / k;

  tmp = (((touchX[0] - touchX[2]) * (displayY[1] - displayY[2])) -
         ((displayY[0] - displayY[2]) * (touchX[1] - touchX[2])));
  out_TransMatrix[4] = ((int64_t)tmp << 16) / k;

  tmp = ((touchY[0] * (((touchX[2] * displayY[1]) - (touchX[1] * displayY[2])))) +
         (touchY[1] * (((touchX[0] * displayY[2]) - (touchX[2] * displayY[0])))) +
         (touchY[2] * (((touchX[1] * displayY[0]) - (touchX[0] * displayY[1])))));
  out_TransMatrix[5] = ((int64_t)tmp << 16) / k;
}

void Calibrate_Fixed(uint32_t width_pixels,
                     uint32_t height_pixels,
                     uint32_t touch_x_max,
                     uint32_t touch_y_max)
{
  uint32_t displayX[3], displayY[3];
  uint32_t touchX[3], touchY[3];
  int32_t TransMatrix[6];

  displayX[0] = 0;
  displayY[0] = 0;
  displayX[1] = width_pixels;
  displayY[1] = 0;
  displayX[2] = width_pixels;
  displayY[2] = height_pixels;

  touchX[0] = 0;
  touchY[0] = 0;
  touchX[1] = touch_x_max;
  touchY[1] = 0;
  touchX[2] = touch_x_max;
  touchY[2] = touch_y_max;

  calculate_touch_matrix(displayX, displayY, touchX, touchY, TransMatrix);
  int count = 0;
  do
  {
    wr32(REG_TOUCH_TRANSFORM_A + RAM_REG + (count * 4),
         TransMatrix[count]); // Write to Eve config registers

    //    uint16_t ValH = TransMatrix[count] >> 16;
    //    uint16_t ValL = TransMatrix[count] & 0xFFFF;
    //    Log("TM%d: 0x%04x %04x\n", count, ValH, ValL);

    count++;
  } while (count < 6);
}
// An interactive calibration screen is created and executed.
// New calibration values are written to the touch matrix registers of Eve.
void Calibrate_Manual(uint16_t Width, uint16_t Height, uint16_t V_Offset, uint16_t H_Offset)
{
  uint32_t displayX[3], displayY[3];
  uint32_t touchX[3], touchY[3];
  uint32_t touchValue = 0, storedValue = 0;
  int32_t tmp, k;
  int32_t TransMatrix[6];
  uint8_t count = 0;
  uint8_t pressed = 0;
  char num[2];

  // These values determine where your calibration points will be drawn on your display
  displayX[0] = (uint32_t)(Width * 0.15) + H_Offset;
  displayY[0] = (uint32_t)(Height * 0.15) + V_Offset;

  displayX[1] = (uint32_t)(Width * 0.85) + H_Offset;
  displayY[1] = (uint32_t)(Height / 2) + V_Offset;

  displayX[2] = (uint32_t)(Width / 2) + H_Offset;
  displayY[2] = (uint32_t)(Height * 0.85) + V_Offset;

  while (count < 3)
  {
    Send_CMD(CMD_DLSTART);
    Send_CMD(CLEAR_COLOR_RGB(0, 0, 0));
    Send_CMD(CLEAR(1, 1, 1));

    // Draw Calibration Point on screen
    Send_CMD(COLOR_RGB(255, 0, 0));
    Send_CMD(POINT_SIZE(20 * 16));
    Send_CMD(BEGIN(POINTS));
    Send_CMD(VERTEX2F((uint32_t)(displayX[count]) * 16, (uint32_t)((displayY[count])) * 16));
    Send_CMD(END());
    Send_CMD(COLOR_RGB(255, 255, 255));
    Cmd_Text((Width / 2) + H_Offset, (Height / 3) + V_Offset, 27, OPT_CENTER, "Calibrating");
    Cmd_Text(
        (Width / 2) + H_Offset, (Height / 2) + V_Offset, 27, OPT_CENTER, "Please tap the dots");
    num[0] = count + 0x31;
    num[1] = 0; // null terminated string of one character
    Cmd_Text(displayX[count], displayY[count], 27, OPT_CENTER, num);

    Send_CMD(DISPLAY());
    Send_CMD(CMD_SWAP);
    UpdateFIFO();          // Trigger the coprocessor to start processing commands out of the FIFO
    Wait4CoProFIFOEmpty(); // Wait here until the coprocessor has read and executed every pending
                           // command.
    HAL_Delay(300);

    while (pressed == count)
    {
      touchValue = rd32(REG_TOUCH_DIRECT_XY + RAM_REG); // Read for any new touch tag inputs
      if (!(touchValue & 0x80000000))
      {
        touchX[count] = (touchValue >> 16) & 0x03FF; // Raw Touchscreen Y coordinate
        touchY[count] = touchValue & 0x03FF;         // Raw Touchscreen Y coordinate

        // Log("\ndisplay x[%d]: %ld display y[%d]: %ld\n", count, displayX[count], count,
        // displayY[count]); Log("touch x[%d]: %ld touch y[%d]: %ld\n", count, touchX[count],
        // count, touchY[count]);

        count++;
      }
    }
    pressed = count;
  }
  calculate_touch_matrix(displayX, displayY, touchX, touchY, TransMatrix);
  count = 0;
  do
  {
    wr32(REG_TOUCH_TRANSFORM_A + RAM_REG + (count * 4),
         TransMatrix[count]); // Write to Eve config registers

    //    uint16_t ValH = TransMatrix[count] >> 16;
    //    uint16_t ValL = TransMatrix[count] & 0xFFFF;
    //    Log("TM%d: 0x%04x %04x\n", count, ValH, ValL);

    count++;
  } while (count < 6);
}
// ***************************************************************************************************************
// *** Animation functions
// ***************************************************************************************
// ***************************************************************************************************************

void Cmd_AnimStart(int32_t ch, uint32_t aoptr, uint32_t loop)
{
  Send_CMD(CMD_ANIMSTART);
  Send_CMD(ch);
  Send_CMD(aoptr);
  Send_CMD(loop);
}

void Cmd_AnimStop(int32_t ch)
{
  Send_CMD(CMD_ANIMSTOP);
  Send_CMD(ch);
}

void Cmd_AnimXY(int32_t ch, int16_t x, int16_t y)
{
  Send_CMD(CMD_ANIMXY);
  Send_CMD(ch);
  Send_CMD(((uint32_t)y << 16) | x);
}

void Cmd_AnimDraw(int32_t ch)
{
  Send_CMD(CMD_ANIMDRAW);
  Send_CMD(ch);
}

void Cmd_AnimDrawFrame(int16_t x, int16_t y, uint32_t aoptr, uint32_t frame)
{
  Send_CMD(CMD_ANIMFRAME);
  Send_CMD(((uint32_t)y << 16) | x);
  Send_CMD(aoptr);
  Send_CMD(frame);
}

// ***************************************************************************************************************
// *** Utility and helper functions
// ******************************************************************************
// ***************************************************************************************************************

// Find the space available in the GPU AKA coprocessor AKA command buffer AKA FIFO
uint16_t CoProFIFO_FreeSpace(void)
{
  uint16_t cmdBufferDiff, cmdBufferRd, cmdBufferWr, retval;

  cmdBufferRd = rd16(REG_CMD_READ + RAM_REG);
  cmdBufferWr = rd16(REG_CMD_WRITE + RAM_REG);

  cmdBufferDiff = (cmdBufferWr - cmdBufferRd) % FT_CMD_FIFO_SIZE; // FT81x Programmers Guide 5.1.1
  retval = (FT_CMD_FIFO_SIZE - 4) - cmdBufferDiff;
  return (retval);
}

// Sit and wait until there are the specified number of bytes free in the <GPU/Coprocessor>
// incoming FIFO
void Wait4CoProFIFO(uint32_t room)
{
  uint16_t getfreespace;

  do
  {
    getfreespace = CoProFIFO_FreeSpace();
  } while (getfreespace < room);
}

// Sit and wait until the CoPro FIFO is empty
// Detect operational errors and print the error and stop.
void Wait4CoProFIFOEmpty(void)
{
  uint16_t ReadReg;
  uint8_t ErrChar;
  do
  {
    ReadReg = rd16(REG_CMD_READ + RAM_REG);
    if (ReadReg == 0xFFF)
    {
      // This is a error which would require sophistication to fix and continue but we fake it
      // somewhat unsuccessfully
      Log("\n");
      uint8_t Offset = 0;
      do
      {
        // Get the error character and display it
        ErrChar = rd8(RAM_ERR_REPORT + Offset);
        Offset++;
        Log("%c", ErrChar);
      } while ((ErrChar != 0) &&
               (Offset < 128)); // When the last stuffed character was null, we are done
      Log("\n");

      // EVE is unhappy - needs a paddling.
      uint32_t Patch_Add = rd32(REG_COPRO_PATCH_PTR + RAM_REG);
      wr8(REG_CPU_RESET + RAM_REG, 1);
      wr16(REG_CMD_READ + RAM_REG, 0);
      wr16(REG_CMD_WRITE + RAM_REG, 0);
      wr16(REG_CMD_DL + RAM_REG, 0);
      wr8(REG_CPU_RESET + RAM_REG, 0);
      wr32(REG_COPRO_PATCH_PTR + RAM_REG, Patch_Add);
      HAL_Delay(250); // We already saw one error message and we don't need to see then 1000 times
                      // a second
    }
  } while (ReadReg != rd16(REG_CMD_WRITE + RAM_REG));
}

// Every CoPro transaction starts with enabling the SPI and sending an address
void StartCoProTransfer(uint32_t address, uint8_t reading)
{
  HAL_SPI_Enable();
  if (reading)
  {
    HAL_SPI_Write(address >> 16);
    HAL_SPI_Write(address >> 8);
    HAL_SPI_Write(address);
    HAL_SPI_Write(0);
  }
  else
  {
    HAL_SPI_Write((address >> 16) | 0x80);
    HAL_SPI_Write(address >> 8);
    HAL_SPI_Write(address);
  }
}

// *** CoProWrCmdBuf() - Transfer a buffer into the CoPro FIFO as part of an ongoing command
// operation ***********
void CoProWrCmdBuf(const uint8_t *buff, uint32_t count)
{
  uint32_t TransferSize = 0;
  int32_t Remaining = count; // Signed

  do
  {
    // Here is the situation:  You have up to about a megabyte of data to transfer into the FIFO
    // Your buffer is LogBuf - limited to 64 bytes (or some other value, but always limited).
    // You need to go around in loops taking 64 bytes at a time until all the data is gone.
    //
    // Most interactions with the FIFO are started and finished in one operation in an obvious
    // fashion, but here it is important to understand the difference between EVE RAM registers and
    // Eve FIFO.  Even though you are in the middle of a FIFO operation and filling the FIFO is an
    // ongoing task, you are still free to write and read non-FIFO registers on the EVE chip.
    //
    // Since the FIFO is 4K in size, but the RAM_G space is 1M in size, you can not, obviously,
    // send all the possible RAM_G data through the FIFO in one step.  Also, since the EVE is not
    // capable of updating it's own FIFO pointer as data is written, you will need to
    // intermittently tell EVE to go process some FIFO in order to make room in the FIFO for more
    // RAM_G data.

    Wait4CoProFIFO(
        WorkBuffSz); // It is reasonable to wait for a small space instead of firing data piecemeal

    if (Remaining > WorkBuffSz)  // Remaining data exceeds the size of our buffer
      TransferSize = WorkBuffSz; // So set the transfer size to that of our buffer
    else
    {
      TransferSize = Remaining;                  // Set size to this last dribble of data
      TransferSize = (TransferSize + 3) & 0xFFC; // 4 byte alignment
    }

    StartCoProTransfer(FifoWriteLocation + RAM_CMD,
                       false); // Base address of the Command Buffer plus our offset into it -
                               // Start SPI transaction

    HAL_SPI_WriteBuffer((uint8_t *)buff,
                        TransferSize); // Write the little bit for which we found space
    buff += TransferSize;              // Move the working data read pointer to the next fresh data

    FifoWriteLocation = (FifoWriteLocation + TransferSize) % FT_CMD_FIFO_SIZE;
    HAL_SPI_Disable(); // End SPI transaction with the FIFO

    wr16(REG_CMD_WRITE + RAM_REG, FifoWriteLocation); // Manually update the write position pointer
                                                      // to initiate processing of the FIFO
    Remaining -= TransferSize;                        // reduce what we want by what we sent

  } while (Remaining > 0); // Keep going as long as we still want more
}

// Write a block of data into EVE RAM space a byte at a time.
// Return the last written address + 1 (The next available RAM address)
uint32_t WriteBlockRAM(uint32_t Add, const uint8_t *buff, uint32_t count)
{
  uint32_t index;
  uint32_t WriteAddress =
      Add; // I want to return the value instead of modifying the variable in place

  for (index = 0; index < count; index++)
  {
    wr8(WriteAddress++, buff[index]);
  }
  return (WriteAddress);
}

// CalcCoef - Support function for manual screen calibration function
int32_t CalcCoef(int32_t Q, int32_t K)
{
  int8_t sn = 0;

  if (Q < 0) // We need to work with positive values
  {
    Q *= -1; // So here we make them positive
    sn++;    // and remember that fact
  }

  if (K < 0)
  {
    K *= -1;
    sn++; // 1 + 1 = 2 = 0b00000010
  }

  uint32_t I = ((uint32_t)Q / (uint32_t)K) << 16; // Get the integer part and shift it by 16
  uint32_t R = Q % K;                             // Get the remainder of a/k;
  R = R << 14;                                    // Shift by 14
  R = R / K;                                      // Divide
  R = R << 2;                                     // Make up for the missing bits
  int32_t returnValue = I + R;                    // Combine them

  if (sn & 0x01)       // If the result is supposed to be negative
    returnValue *= -1; // then return it to that state.

  return (returnValue);
}

bool FlashAttach(void)
{
  Send_CMD(CMD_FLASHATTACH);
  UpdateFIFO();          // Trigger the coprocessor to start processing commands out of the FIFO
  Wait4CoProFIFOEmpty(); // Wait here until the coprocessor has read and executed every pending
                         // command.

  uint8_t FlashStatus = rd8(REG_FLASH_STATUS + RAM_REG);
  if (FlashStatus != FLASH_STATUS_BASIC)
  {
    return false;
  }
  return true;
}

bool FlashDetach(void)
{
  Send_CMD(CMD_FLASHDETACH);
  UpdateFIFO();          // Trigger the coprocessor to start processing commands out of the FIFO
  Wait4CoProFIFOEmpty(); // Wait here until the coprocessor has read and executed every pending
                         // command.

  uint8_t FlashStatus = rd8(REG_FLASH_STATUS + RAM_REG);
  if (FlashStatus != FLASH_STATUS_DETACHED)
  {
    return false;
  }
  return true;
}

bool FlashFast(void)
{
  Cmd_Flash_Fast();
  UpdateFIFO();          // Trigger the coprocessor to start processing commands out of the FIFO
  Wait4CoProFIFOEmpty(); // Wait here until the coprocessor has read and executed every pending
                         // command.

  uint8_t FlashStatus = rd8(REG_FLASH_STATUS + RAM_REG);
  if (FlashStatus != FLASH_STATUS_FULL)
  {
    return false;
  }
  return true;
}

bool FlashErase(void)
{
  Send_CMD(CMD_FLASHERASE);
  UpdateFIFO();          // Trigger the coprocessor to start processing commands out of the FIFO
  Wait4CoProFIFOEmpty(); // Wait here until the coprocessor has read and executed every pending
                         // command.
  return true;
}

void UploadTouchFirmware(const uint8_t *firmware, size_t length)
{
  CoProWrCmdBuf(firmware, length);
  UpdateFIFO();
  Wait4CoProFIFOEmpty();
  wr8(REG_CPU_RESET + RAM_REG, 2);
  wr8(REG_GPIOX_DIR + RAM_REG, (rd8(RAM_REG + REG_GPIOX_DIR) | 0x08)); // Set Disp GPIO Direction
  wr8(REG_GPIOX + RAM_REG, (rd8(RAM_REG + REG_GPIOX) | 0xF7));         // Clear GPIO
  HAL_Delay(1);
  wr8(REG_CPU_RESET + RAM_REG, 0);
  HAL_Delay(100);
  wr8(REG_GPIOX_DIR + RAM_REG, (rd8(RAM_REG + REG_GPIOX_DIR) & 0xF7)); // Set Disp GPIO Direction
}

#if defined(EVE_MO_INTERNAL_BUILD)
void EVE_SPI_Enable(void)
{
  HAL_SPI_Enable();
}

void EVE_SPI_Disable(void)
{
  HAL_SPI_Disable();
}

uint8_t EVE_SPI_Write(uint8_t data)
{
  return HAL_SPI_Write(data);
}

void EVE_SPI_WriteBuffer(uint8_t *Buffer, uint32_t Length)
{
  HAL_SPI_WriteBuffer(Buffer, Length);
}
#endif
