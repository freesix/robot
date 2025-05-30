#pragma once

typedef enum{
	NULL_RETURN,
	MIDDLE_RETURN,
	CLEAR_BUF_RETURN,
	END_RETURN
}DisposeFlag;

unsigned int Crc32Check(unsigned char *buf, unsigned int size);
unsigned char FindString_token(unsigned char **data, unsigned int *len, char token, char *v, unsigned int max_len);
unsigned char* FindString(unsigned char* Data, const unsigned int DataLen, unsigned char* StringVal, const unsigned int StringLen);
DisposeFlag FindDispose(unsigned char* Data, const unsigned int DataLen, unsigned char* StringVal,unsigned char** SAddr, unsigned int* OKLen);