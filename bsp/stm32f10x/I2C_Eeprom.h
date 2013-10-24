
#ifndef _I2C_EEPROM_H_
#define _I2C_EEPROM_H_

typedef uint8_t BYTE;
typedef uint8_t*        PBYTE;
typedef uint16_t        WORD;
typedef int16_t         SWORD;
typedef uint32_t        DWORD;
typedef int32_t         SDWORD;
typedef int8_t          CHAR;
typedef float           FLOAT;
typedef BYTE            BIT;

#define CONST           const
#define VOID void

typedef enum {FALSE = 0, TRUE = !FALSE} bool;

/* E2 IC 型号选择 */
#define USE_24C01   1
#define USE_24C02   2
#define USE_24C04   3
#define USE_24C08   4
#define USE_24C16   5
#define USE_24C32   6
#define USE_24C64   7
#define USE_24C128  8
#define USE_24C256  9
#define USE_24C512  10

#define USE_E2IC_TYPE   USE_24C256  /* 选择IC型号 */

#if USE_E2IC_TYPE <= USE_24C16
	#if   USE_E2IC_TYPE == USE_24C01
		#define E2END		0x007F
		#define E2PAGESIZE	16
	#elif USE_E2IC_TYPE == USE_24C02
		#define E2END		0x00FF
		#define E2PAGESIZE	16
	#elif USE_E2IC_TYPE == USE_24C04
		#define E2END		0x01FF
		#define E2PAGESIZE	16
		#define SLADD_MASK	0x01
	#elif USE_E2IC_TYPE == USE_24C08
		#define E2END		0x03FF
		#define E2PAGESIZE	16
		#define SLADD_MASK	0x03
	#elif USE_E2IC_TYPE == USE_24C16
		#define E2END		0x07FF
		#define E2PAGESIZE	16
		#define SLADD_MASK	0x07
	#else
	#endif
#else
	#if   USE_E2IC_TYPE == USE_24C32
		#define E2END		0x0FFF
		#define E2PAGESIZE	32
	#elif USE_E2IC_TYPE == USE_24C64
		#define E2END		0x1FFF
		#define E2PAGESIZE	32
	#elif USE_E2IC_TYPE == USE_24C128
		#define E2END		0x3FFF
		#define E2PAGESIZE	64
	#elif USE_E2IC_TYPE == USE_24C256
		#define E2END		0x7FFF
		#define E2PAGESIZE	64
	#elif USE_E2IC_TYPE == USE_24C512
		#define E2END		0xFFFF
		#define E2PAGESIZE	64
	#else
	#endif
	#define USE_SUBADDR16B	1	/* add cql 20100810 */
#endif

#define E2_SLAVEADDR 0xA0   /* E2 地址， cql modify 20100921 */
#define E2_PORTNUM	 1

VOID I2c_Soft_Init(BYTE byI2cPort);
VOID I2c_Soft_E2IcTypeSet(BYTE byE2IcTyp);
BYTE I2c_Soft_Write(BYTE byI2cPort, BYTE bySlave, BYTE byAddr, BYTE *pbyData);
BYTE I2c_Soft_Read(BYTE byI2cPort, BYTE bySlave, BYTE byAddr, BYTE *pbyData);
//BYTE I2c_Soft_BufferWrite(BYTE byI2cPort, BYTE bySlave, WORD WriteAddr, BYTE* pBuffer, BYTE NumByteToWrite);
BYTE I2c_Soft_BufferWrite(BYTE byI2cPort, BYTE bySlave, WORD WriteAddr, BYTE* pBuffer, WORD NumByteToWrite);
BYTE I2c_Soft_BufferRead(BYTE byI2cPort, BYTE bySlave, WORD WriteAddr, BYTE* pBuffer, WORD NumByteToRead);
VOID I2c_Soft_WaitE2StandBy(BYTE byI2cPort, BYTE bySlave);

BYTE I2c_Soft_E2IcTypeDet(BYTE byI2cPort, BYTE bySlave);	/* cql cdd for E2 Type det */

BYTE ReadByte(WORD wAddrS);
VOID WriteByte(WORD wAddrS, BYTE byValue);
DWORD ReadDword(WORD wAddrS);
VOID WriteDWord(WORD wAddrS, DWORD wValue);
VOID WriteBlock(WORD wAddrS, BYTE *pbuf, WORD wbufSize);
VOID ReadBlock(WORD wAddrS, BYTE *pbuf, WORD wbufSize);

#endif

