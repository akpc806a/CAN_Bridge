//---------------------------------------------------------------------------

#ifndef InterfaceH
#define InterfaceH

#include <windows.h>

//---------------------------------------------------------------------------
#define PORT_BUFFER_COUNT 16384

#define HAL_ERROR_OK 0
#define HAL_ERROR_FATAL 256
#define HAL_ERROR_NO_CONNECT 257
#define HAL_ERROR_IO 258
#define HAL_ERROR_CMD 259
#define HAL_ERROR_PARAMS 260
#define HAL_ERROR_XBEE 261
#define HAL_ERROR_NO_INI_FILE 262
//---------------------------------------------------------------------------


class TCOMCtrl
{
private:
  HANDLE hCOMPort;

public:
  int HAL_InData(unsigned char *Data, int Count);
  int HAL_OutData(unsigned char *Data, int Count);

  int HAL_Init(unsigned char Index, unsigned int Speed);

  int HAL_Close();
  int HAL_GetInDataCount();
  void HAL_ResetIn();
};
//---------------------------------------------------------------------------
#endif
