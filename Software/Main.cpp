//---------------------------------------------------------------------------

#include <vcl.h>
#pragma hdrstop

#include "Main.h"
#include <Registry.hpp>
#include <stdio.h>
#include <cstdlib> 
//---------------------------------------------------------------------------
#pragma package(smart_init)
#pragma resource "*.dfm"
TMainForm *MainForm;

char bRxBuffer[PORT_BUFFER_COUNT];
char bTxBuffer[PORT_BUFFER_COUNT];

#define MAX_FILTER_COUNT 40

unsigned __int64 strtoull(const char * nptr, char ** endptr, int base);
unsigned __int64 revert_bytes(unsigned __int64 x);
AnsiString NoSpaces(AnsiString s);
//---------------------------------------------------------------------------
__fastcall TMainForm::TMainForm(TComponent* Owner)
        : TForm(Owner)
{
  InterfaceCtrl = new TCOMCtrl;
}

void GetSerialPortsList()
{
    AnsiString KeyName = "\\Hardware\\DeviceMap\\SerialComm";
    TStringList *SerialCommValues = new TStringList();
    MainForm->ComboBox_COMPorts->Items->Clear();
    TRegistry *Registry = new TRegistry;
    try
    {
        Registry->RootKey = HKEY_LOCAL_MACHINE;
        Registry->OpenKeyReadOnly( KeyName );
        Registry->GetValueNames( SerialCommValues );
        for(int i=0; i<SerialCommValues->Count; i++)
        {
            MainForm->ComboBox_COMPorts->Items->Add(Registry->ReadString(SerialCommValues->Strings[i]));
        }
    }
    __finally
    {
        delete Registry;
        delete SerialCommValues;
        if (MainForm->ComboBox_COMPorts->ItemIndex<0)
        {
            MainForm->ComboBox_COMPorts->ItemIndex=0;
        }
    }
}


//---------------------------------------------------------------------------


void __fastcall TMainForm::FormShow(TObject *Sender)
{
  GetSerialPortsList();        
}
//---------------------------------------------------------------------------
void __fastcall TMainForm::ComboBox_COMPortsDropDown(TObject *Sender)
{
  GetSerialPortsList();         
}
//---------------------------------------------------------------------------
void __fastcall TMainForm::Button_OpenClick(TObject *Sender)
{
  AnsiString sName = ComboBox_COMPorts->Text;
  sName = sName.SubString(4, sName.Length()-3);
  int iPort = sName.ToIntDef(-1);
  if (iPort < 0)
  {
    ShowMessage("Incorrect port name");
    return;
  }
  int iRes = InterfaceCtrl->HAL_Init(iPort, 115200);
  if (iRes != HAL_ERROR_OK)
  {
    ShowMessage("Failed to open port(");
    return;
  }

  Button_Write->Enabled = true;
  Button_Read->Enabled = true;
}
//---------------------------------------------------------------------------
void Int64ToByteStr(char * s, unsigned __int64* data)
{
  unsigned char *bBytes;
  bBytes = (unsigned char *)data;
  sprintf(s, "%02X %02X %02X %02X %02X %02X %02X %02X", bBytes[4], bBytes[5], bBytes[6], bBytes[7], bBytes[0], bBytes[1], bBytes[2], bBytes[3]);
}
//---------------------------------------------------------------------------
void __fastcall TMainForm::Button_ReadClick(TObject *Sender)
{
  int iAddr = 0x800F800;
  int iLength = 40 + 4; // CAN parameters and replacement count

  // read command
  char *bBuffer; bBuffer = bTxBuffer;
  *bBuffer = '1'; bBuffer++;
  *bBuffer = '2'; bBuffer++;
  *bBuffer = iAddr >> 24; bBuffer++;
  *bBuffer = iAddr >> 16; bBuffer++;
  *bBuffer = iAddr >> 8; bBuffer++;
  *bBuffer = iAddr; bBuffer++;
  *bBuffer = iLength >> 8; bBuffer++;
  *bBuffer = iLength; bBuffer++;
  *bBuffer = 'R'; bBuffer++;

  if (InterfaceCtrl->HAL_OutData(bTxBuffer, (int)(bBuffer-bTxBuffer)) != HAL_ERROR_OK)
  {
    ShowMessage("Failed to send data");
    return;
  }

  if (InterfaceCtrl->HAL_InData(bRxBuffer, iLength) != HAL_ERROR_OK)
  {
    ShowMessage("Failed to read data");
    return;
  }

  int *iBuffer;
  iBuffer = (int*)bRxBuffer;
  int iCAN1_Prescaler         = *iBuffer; iBuffer++;
  int iCAN2_Prescaler         = *iBuffer; iBuffer++;
  int iCAN1_FilterIdHigh      = *iBuffer; iBuffer++;
  int iCAN1_FilterIdLow       = *iBuffer; iBuffer++;
  int iCAN1_FilterMaskIdHigh  = *iBuffer; iBuffer++;
  int iCAN1_FilterMaskIdLow   = *iBuffer; iBuffer++;
  int iCAN2_FilterIdHigh      = *iBuffer; iBuffer++;
  int iCAN2_FilterIdLow       = *iBuffer; iBuffer++;
  int iCAN2_FilterMaskIdHigh  = *iBuffer; iBuffer++;
  int iCAN2_FilterMaskIdLow   = *iBuffer; iBuffer++;

  ComboBox_Baudrate1->Text = IntToStr(int(1000*3.0/float(iCAN1_Prescaler)  + 0.5));
  ComboBox_Baudrate2->Text = IntToStr(int(1000*3.0/float(iCAN2_Prescaler)  + 0.5));

  char sTmp[50];
  //sprintf(sTmp, "%08X%08X", iCAN1_FilterIdHigh, iCAN1_FilterIdLow);
  sprintf(sTmp, "%08X", iCAN1_FilterIdLow);
  Edit_FilterId1->Text = sTmp;
  //sprintf(sTmp, "%08X%08X", iCAN1_FilterMaskIdHigh, iCAN1_FilterMaskIdLow);
  sprintf(sTmp, "%08X", iCAN1_FilterMaskIdLow);
  Edit_FilterMask1->Text = sTmp;
  //sprintf(sTmp, "%08X%08X", iCAN2_FilterIdHigh, iCAN2_FilterIdLow);
  sprintf(sTmp, "%08X", iCAN2_FilterIdLow);
  Edit_FilterId2->Text = sTmp;
  //sprintf(sTmp, "%08X%08X", iCAN2_FilterMaskIdHigh, iCAN2_FilterMaskIdLow);
  sprintf(sTmp, "%08X", iCAN2_FilterMaskIdLow);
  Edit_FilterMask2->Text = sTmp;

  unsigned int iReplaceCount = *iBuffer;

  if (iReplaceCount > MAX_FILTER_COUNT) iReplaceCount = MAX_FILTER_COUNT;

  if (iReplaceCount == 0)
  {
    // no replacements in memory -- empty grid
    StringGrid->RowCount = 2;
    for (int j = 1; j < StringGrid->ColCount; j++)
    {
      StringGrid->Cells[j][1] = "";
    }
    return; // no replacements in memory -- exit
  }
  StringGrid->RowCount = iReplaceCount + 1;

  // read command
  iAddr = iAddr + iLength;

  for (int i = 0; i < iReplaceCount; i++)
  {
    iLength = (4*4 + 4*8); // one record length

    bBuffer = bTxBuffer;
    *bBuffer = '1'; bBuffer++;
    *bBuffer = '2'; bBuffer++;
    *bBuffer = iAddr >> 24; bBuffer++;
    *bBuffer = iAddr >> 16; bBuffer++;
    *bBuffer = iAddr >> 8; bBuffer++;
    *bBuffer = iAddr; bBuffer++;
    *bBuffer = iLength >> 8; bBuffer++;
    *bBuffer = iLength; bBuffer++;
    *bBuffer = 'R'; bBuffer++;

    if (InterfaceCtrl->HAL_OutData(bTxBuffer, (int)(bBuffer-bTxBuffer)) != HAL_ERROR_OK)
    {
      ShowMessage("Failed to send data");
      return;
    }

    if (InterfaceCtrl->HAL_InData(bRxBuffer, iLength) != HAL_ERROR_OK)
    {
      ShowMessage("Failed to read data");
      return;
    }


    iBuffer = (int*)bRxBuffer;

    // "ID Mask";
    sprintf(sTmp, "%08X", *iBuffer);
    StringGrid->Cells[1][i+1] = sTmp;
    iBuffer++;
    // "ID Filter"
    sprintf(sTmp, "%08X", *iBuffer);
    StringGrid->Cells[2][i+1] = sTmp;
    iBuffer++;
    // "New ID Mask"
    sprintf(sTmp, "%08X", *iBuffer);
    StringGrid->Cells[3][i+1] = sTmp;
    iBuffer++;
    // "New ID Value"
    sprintf(sTmp, "%08X", *iBuffer);
    StringGrid->Cells[4][i+1] = sTmp;
    iBuffer++;

    // "Data Mask"
    Int64ToByteStr(sTmp, (unsigned __int64*)iBuffer);
    StringGrid->Cells[5][i+1] = sTmp;
    iBuffer+=2;

    // "Data Filter"
    Int64ToByteStr(sTmp, (unsigned __int64*)iBuffer);
    StringGrid->Cells[6][i+1] = sTmp;
    iBuffer+=2;

    // "New Data Mask"
    //sprintf(sTmp, "%08X%08X", *iBuffer, *(iBuffer+1));
    Int64ToByteStr(sTmp, (unsigned __int64*)iBuffer);
    StringGrid->Cells[7][i+1] = sTmp;
    iBuffer+=2;

    // "New Data Value"
    //sprintf(sTmp, "%08X%08X", *iBuffer, *(iBuffer+1));
    Int64ToByteStr(sTmp, (unsigned __int64*)iBuffer);
    StringGrid->Cells[8][i+1] = sTmp;
    iBuffer+=2;

    iAddr = iAddr + iLength;
  }
}
//---------------------------------------------------------------------------
#define ULLONG_MAX	((unsigned __int64)(18446744073709551615))

unsigned __int64
strtoull(const char * nptr, char ** endptr, int base)
{
	const char *s;
	unsigned __int64 acc;
	char c;
	unsigned __int64 cutoff;
	int neg, any, cutlim;

	/*
	 * See strtoq for comments as to the logic used.
	 */
	s = nptr;
	do {
		c = *s++;
	} while (isspace((unsigned char)c));
	if (c == '-') {
		neg = 1;
		c = *s++;
	} else {
		neg = 0;
		if (c == '+')
			c = *s++;
	}
	if ((base == 0 || base == 16) &&
	    c == '0' && (*s == 'x' || *s == 'X')) {
		c = s[1];
		s += 2;
		base = 16;
	}
	if (base == 0)
		base = c == '0' ? 8 : 10;
	acc = any = 0;
	if (base < 2 || base > 36)
		goto noconv;

	cutoff = ULLONG_MAX / base;
	cutlim = ULLONG_MAX % base;
	for ( ; ; c = *s++) {
		if (c >= '0' && c <= '9')
			c -= '0';
		else if (c >= 'A' && c <= 'Z')
			c -= 'A' - 10;
		else if (c >= 'a' && c <= 'z')
			c -= 'a' - 10;
		else
			break;
		if (c >= base)
			break;
		if (any < 0 || acc > cutoff || (acc == cutoff && c > cutlim))
			any = -1;
		else {
			any = 1;
			acc *= base;
			acc += c;
		}
	}
	if (any < 0) {
		acc = ULLONG_MAX;
		errno = ERANGE;
	} else if (!any) {
noconv:
		errno = EINVAL;
	} else if (neg)
		acc = -acc;
	if (endptr != NULL)
		*endptr = (char *)(any ? s - 1 : nptr);
	return (acc);
}

unsigned __int64 revert_bytes(unsigned __int64 x)
{
  char sTmp[32];
  // this is hodgie code style conversion :)
  Int64ToByteStr(sTmp, &x);
  char* sEnd = 0;
  unsigned __int64 y = 0;
  y = strtoull(AnsiString("0x"+NoSpaces(sTmp)).c_str(), &sEnd, 16);

  return y;
}
//---------------------------------------------------------------------------

void __fastcall TMainForm::Button_WriteClick(TObject *Sender)
{
  int iAddr = 0x800F800;
  int iLength = 40 + 4; // CAN parameters and replacement records count

  // check GUI data

  if (ComboBox_Baudrate1->Text.ToIntDef(-1) <= 0) { ShowMessage("Check CAN1 baudrate value"); return; }
  if (ComboBox_Baudrate2->Text.ToIntDef(-1) <= 0) { ShowMessage("Check CAN2 baudrate value"); return; }
  
  unsigned __int64 iValue;
  char* sEnd = 0;

  iValue = strtoull(AnsiString("0x"+Edit_FilterId1->Text).c_str(), &sEnd, 16);
  if (*sEnd != 0) { ShowMessage("Check CAN1 filter ID value"); return; }
  iValue = strtoull(AnsiString("0x"+Edit_FilterMask1->Text).c_str(), &sEnd, 16);
  if (*sEnd != 0) { ShowMessage("Check CAN1 filter mask value"); return; }

  iValue = strtoull(AnsiString("0x"+Edit_FilterId2->Text).c_str(), &sEnd, 16);
  if (*sEnd != 0) { ShowMessage("Check CAN2 filter ID value"); return; }
  iValue = strtoull(AnsiString("0x"+Edit_FilterMask2->Text).c_str(), &sEnd, 16);
  if (*sEnd != 0) { ShowMessage("Check CAN2 filter mask value"); return; }


  if (ValidateStringGridContents() == 0) return;


  char *bBuffer;

  // Erase flash command
  
  bBuffer = bTxBuffer;
  *bBuffer = '1'; bBuffer++;
  *bBuffer = '2'; bBuffer++;
  *bBuffer = iAddr >> 24; bBuffer++;
  *bBuffer = iAddr >> 16; bBuffer++;
  *bBuffer = iAddr >> 8; bBuffer++;
  *bBuffer = iAddr; bBuffer++;
  *bBuffer = iLength >> 8; bBuffer++;
  *bBuffer = iLength; bBuffer++;
  *bBuffer = 'F'; bBuffer++;

  if (InterfaceCtrl->HAL_OutData(bTxBuffer, (int)(bBuffer-bTxBuffer)) != HAL_ERROR_OK)
  {
    ShowMessage("Failed to send data");
    return;
  }

  if (InterfaceCtrl->HAL_InData(bRxBuffer, 3) != HAL_ERROR_OK)
  {
    ShowMessage("Failed to read data");
    return;
  }
  if (bRxBuffer[0] != '1' || bRxBuffer[1] != '2' || bRxBuffer[2] != 'Y')
  {
    ShowMessage("Erase operation failed");
    return;
  }

  // Writing CAN parameters

  // write command
  bBuffer = bTxBuffer;
  *bBuffer = '1'; bBuffer++;
  *bBuffer = '2'; bBuffer++;
  *bBuffer = iAddr >> 24; bBuffer++;
  *bBuffer = iAddr >> 16; bBuffer++;
  *bBuffer = iAddr >> 8; bBuffer++;
  *bBuffer = iAddr; bBuffer++;
  *bBuffer = iLength >> 8; bBuffer++;
  *bBuffer = iLength; bBuffer++;
  *bBuffer = 'W'; bBuffer++;

  if (InterfaceCtrl->HAL_OutData(bTxBuffer, (int)(bBuffer-bTxBuffer)) != HAL_ERROR_OK)
  {
    ShowMessage("Failed to send data");
    return;
  }



  // prepare CAN parameters data
  unsigned __int64 iCAN1_FilterId = strtoull(AnsiString("0x"+Edit_FilterId1->Text).c_str(), &sEnd, 16);
  unsigned __int64 iCAN1_FilterMaskId = strtoull(AnsiString("0x"+Edit_FilterMask1->Text).c_str(), &sEnd, 16);

  unsigned __int64 iCAN2_FilterId = strtoull(AnsiString("0x"+Edit_FilterId2->Text).c_str(), &sEnd, 16);
  unsigned __int64 iCAN2_FilterMaskId = strtoull(AnsiString("0x"+Edit_FilterMask2->Text).c_str(), &sEnd, 16);

  
  int iCAN1_Prescaler         = (1000.0*3/float(ComboBox_Baudrate1->Text.ToIntDef(-1)) + 0.5);
  int iCAN2_Prescaler         = (1000.0*3/float(ComboBox_Baudrate2->Text.ToIntDef(-1)) + 0.5);
  int iCAN1_FilterIdHigh      = iCAN1_FilterId >> 32;
  int iCAN1_FilterIdLow       = iCAN1_FilterId;
  int iCAN1_FilterMaskIdHigh  = iCAN1_FilterMaskId >> 32;
  int iCAN1_FilterMaskIdLow   = iCAN1_FilterMaskId;
  int iCAN2_FilterIdHigh      = iCAN2_FilterId >> 32;
  int iCAN2_FilterIdLow       = iCAN2_FilterId;
  int iCAN2_FilterMaskIdHigh  = iCAN2_FilterMaskId >> 32;
  int iCAN2_FilterMaskIdLow   = iCAN2_FilterMaskId;

  // replacement count
  int iReplaceCount = StringGrid->RowCount-1;
  if (IsRowEmpty(1) && StringGrid->RowCount == 2) iReplaceCount = 0; // this is empty grid

  // write data
  int *iBuffer;
  iBuffer = (int*)bTxBuffer;

  *iBuffer = iCAN1_Prescaler; iBuffer++;
  *iBuffer = iCAN2_Prescaler; iBuffer++;
  *iBuffer = iCAN1_FilterIdHigh; iBuffer++;
  *iBuffer = iCAN1_FilterIdLow; iBuffer++;
  *iBuffer = iCAN1_FilterMaskIdHigh; iBuffer++;
  *iBuffer = iCAN1_FilterMaskIdLow; iBuffer++;
  *iBuffer = iCAN2_FilterIdHigh; iBuffer++;
  *iBuffer = iCAN2_FilterIdLow; iBuffer++;
  *iBuffer = iCAN2_FilterMaskIdHigh; iBuffer++;
  *iBuffer = iCAN2_FilterMaskIdLow; iBuffer++;
  *iBuffer = iReplaceCount; iBuffer++;

  if (InterfaceCtrl->HAL_OutData(bTxBuffer, (int)(((char*)iBuffer)-bTxBuffer)) != HAL_ERROR_OK)
  {
    ShowMessage("Failed to send data");
    return;
  }

  if (InterfaceCtrl->HAL_InData(bRxBuffer, 3) != HAL_ERROR_OK)
  {
    ShowMessage("Failed to read data");
    return;
  }
  if (bRxBuffer[0] != '1' || bRxBuffer[1] != '2' || bRxBuffer[2] != 'Y')
  {
    ShowMessage("Programming operation failed");
    return;
  }


  // Write replacement data
  if (iReplaceCount == 0) return;

  iAddr = iAddr + iLength; // replacement data are just after CANs configuration

  for (int i = 0; i < iReplaceCount; i++)
  {
    iLength = (4*4 + 4*8); // on record length

    // write command
    bBuffer = bTxBuffer;
    *bBuffer = '1'; bBuffer++;
    *bBuffer = '2'; bBuffer++;
    *bBuffer = iAddr >> 24; bBuffer++;
    *bBuffer = iAddr >> 16; bBuffer++;
    *bBuffer = iAddr >> 8; bBuffer++;
    *bBuffer = iAddr; bBuffer++;
    *bBuffer = iLength >> 8; bBuffer++;
    *bBuffer = iLength; bBuffer++;
    *bBuffer = 'W'; bBuffer++;

    if (InterfaceCtrl->HAL_OutData(bTxBuffer, (int)(bBuffer-bTxBuffer)) != HAL_ERROR_OK)
    {
      ShowMessage("Failed to send data");
      return;
    }

    // write data
    iBuffer = (int*)bTxBuffer;

    iValue = strtoull(AnsiString("0x"+NoSpaces(StringGrid->Cells[1][i+1])).c_str(), &sEnd, 16); // "ID Mask";
    *iBuffer = iValue; iBuffer++;
    iValue = strtoull(AnsiString("0x"+NoSpaces(StringGrid->Cells[2][i+1])).c_str(), &sEnd, 16); // "ID Filter"
    *iBuffer = iValue; iBuffer++;
    iValue = strtoull(AnsiString("0x"+NoSpaces(StringGrid->Cells[3][i+1])).c_str(), &sEnd, 16); // "New ID Mask"
    *iBuffer = iValue; iBuffer++;
    iValue = strtoull(AnsiString("0x"+NoSpaces(StringGrid->Cells[4][i+1])).c_str(), &sEnd, 16); // "New ID Value"
    *iBuffer = iValue; iBuffer++;

    iValue = strtoull(AnsiString("0x"+NoSpaces(StringGrid->Cells[5][i+1])).c_str(), &sEnd, 16); // "Data Mask"
    iValue = revert_bytes(iValue);
    *iBuffer = iValue; iBuffer++;
    *iBuffer = iValue >> 32; iBuffer++;
    iValue = strtoull(AnsiString("0x"+NoSpaces(StringGrid->Cells[6][i+1])).c_str(), &sEnd, 16); // "Data Filter"
    iValue = revert_bytes(iValue);
    *iBuffer = iValue; iBuffer++;
    *iBuffer = iValue >> 32; iBuffer++;
    iValue = strtoull(AnsiString("0x"+NoSpaces(StringGrid->Cells[7][i+1])).c_str(), &sEnd, 16); // "New Data Mask"
    iValue = revert_bytes(iValue);
    *iBuffer = iValue; iBuffer++;
    *iBuffer = iValue >> 32; iBuffer++;
    iValue = strtoull(AnsiString("0x"+NoSpaces(StringGrid->Cells[8][i+1])).c_str(), &sEnd, 16); // "New Data Value"
    iValue = revert_bytes(iValue);
    *iBuffer = iValue; iBuffer++;
    *iBuffer = iValue >> 32; iBuffer++;

    if (InterfaceCtrl->HAL_OutData(bTxBuffer, (int)(((char*)iBuffer)-bTxBuffer)) != HAL_ERROR_OK)
    {
      ShowMessage("Failed to send data");
      return;
    }

    if (InterfaceCtrl->HAL_InData(bRxBuffer, 3) != HAL_ERROR_OK)
    {
      ShowMessage("Failed to read data");
      return;
    }
    if (bRxBuffer[0] != '1' || bRxBuffer[1] != '2' || bRxBuffer[2] != 'Y')
    {
      ShowMessage("Programming operation failed");
      return;
    }

    iAddr = iAddr + iLength;
  }




}
//---------------------------------------------------------------------------
void __fastcall TMainForm::FormCreate(TObject *Sender)
{
  // packed is accpeted for replacment if ID and Data are both satisfied their filter condition
  StringGrid->Cells[1][0] = "ID Mask"; // if bit =1, then this bit is being checked for filtering
  StringGrid->Cells[2][0] = "ID Filter"; // if bit in "ID Mask" =1 AND bit in recieved ID = bit in "ID Filter", then packed ID is accepted for modification
  StringGrid->Cells[3][0] = "New ID Mask"; // if bit =1 then the value of this bit is being replaced by bit from "New ID Value"
  StringGrid->Cells[4][0] = "New ID Value";

  StringGrid->Cells[5][0] = "Data Mask [D0 .. D7]";
  StringGrid->Cells[6][0] = "Data Filter [D0 .. D7]"; // if bit in "Data Mask" =1 AND bit in recieved ID = bit in "Data Filter", then packed data is accepted for modification
  StringGrid->Cells[7][0] = "New Data Mask [D0 .. D7]"; // if bit =1 then the value of this bit is being replaced by bit from "New Data Value"
  StringGrid->Cells[8][0] = "New Data Value [D0 .. D7]";

  StringGrid->ColWidths[0] = 20;
  StringGrid->ColWidths[1] = 70;
  StringGrid->ColWidths[2] = 70;
  StringGrid->ColWidths[3] = 70;
  StringGrid->ColWidths[4] = 70;

  StringGrid->Cells[0][StringGrid->RowCount-1] = 1;
}
//---------------------------------------------------------------------------


void __fastcall TMainForm::Button_RowAddClick(TObject *Sender)
{
  if (StringGrid->RowCount-1 >= MAX_FILTER_COUNT) return;
  StringGrid->RowCount++;
  StringGrid->Cells[0][StringGrid->RowCount-1] = StringGrid->RowCount-1;
}
//---------------------------------------------------------------------------


void __fastcall TMainForm::Button_SaveClick(TObject *Sender)
{
  if (SaveDialog->Execute())
  {
    TIniFile *fileOut = new TIniFile(SaveDialog->FileName);
    fileOut->WriteString("CAN1", "Speed", ComboBox_Baudrate1->Text);
    fileOut->WriteString("CAN1", "FilterId", Edit_FilterId1->Text);
    fileOut->WriteString("CAN1", "FilterMask", Edit_FilterMask1->Text);
    fileOut->WriteString("CAN2", "Speed", ComboBox_Baudrate2->Text);
    fileOut->WriteString("CAN2", "FilterId", Edit_FilterId2->Text);
    fileOut->WriteString("CAN2", "FilterMask", Edit_FilterMask2->Text);

    fileOut->WriteInteger("Replace", "Count", StringGrid->RowCount-1);

    for (int i = 0; i < StringGrid->RowCount-1; i++)
    {
      fileOut->WriteString("Replace"+IntToStr(i), "IDMask", StringGrid->Cells[1][i+1]);
      fileOut->WriteString("Replace"+IntToStr(i), "IDFilter", StringGrid->Cells[2][i+1]);
      fileOut->WriteString("Replace"+IntToStr(i), "NewIDMask", StringGrid->Cells[3][i+1]);
      fileOut->WriteString("Replace"+IntToStr(i), "NewIDValue", StringGrid->Cells[4][i+1]);

      fileOut->WriteString("Replace"+IntToStr(i), "DataMask", StringGrid->Cells[5][i+1]);
      fileOut->WriteString("Replace"+IntToStr(i), "DataFilter", StringGrid->Cells[6][i+1]);
      fileOut->WriteString("Replace"+IntToStr(i), "NewDataMask", StringGrid->Cells[7][i+1]);
      fileOut->WriteString("Replace"+IntToStr(i), "NewDataValue", StringGrid->Cells[8][i+1]);
    }

    delete fileOut;
  }
}
//---------------------------------------------------------------------------

void __fastcall TMainForm::Button_LoadClick(TObject *Sender)
{
  if (OpenDialog->Execute())
  {
    TIniFile *fileOut = new TIniFile(OpenDialog->FileName);
    ComboBox_Baudrate1->Text = fileOut->ReadString("CAN1", "Speed", "500");
    Edit_FilterId1->Text = fileOut->ReadString("CAN1", "FilterId", "0");
    Edit_FilterMask1->Text = fileOut->ReadString("CAN1", "FilterMask", "0");
    ComboBox_Baudrate2->Text = fileOut->ReadString("CAN2", "Speed", "500");
    Edit_FilterId2->Text = fileOut->ReadString("CAN2", "FilterId", "0");
    Edit_FilterMask2->Text = fileOut->ReadString("CAN2", "FilterMask", "0");

    int iReplaceCount = fileOut->ReadInteger("Replace", "Count", 0);
    if (iReplaceCount > MAX_FILTER_COUNT)
    {
      AnsiString sMessage = "Configuration file has more replacing records (" + IntToStr(iReplaceCount) + ") than the limit (" + IntToStr(MAX_FILTER_COUNT) +")";
      Application->MessageBox(sMessage.c_str(), "Replacing item list truncated", MB_OK | MB_ICONWARNING);
      iReplaceCount = MAX_FILTER_COUNT;
    }
    StringGrid->RowCount = iReplaceCount + 1;

    for (int i = 0; i < iReplaceCount; i++)
    {
      StringGrid->Cells[1][i+1] = fileOut->ReadString("Replace"+IntToStr(i), "IDMask", "");
      StringGrid->Cells[2][i+1] = fileOut->ReadString("Replace"+IntToStr(i), "IDFilter", "");
      StringGrid->Cells[3][i+1] = fileOut->ReadString("Replace"+IntToStr(i), "NewIDMask", "");
      StringGrid->Cells[4][i+1] = fileOut->ReadString("Replace"+IntToStr(i), "NewIDValue", "");

      StringGrid->Cells[5][i+1] = fileOut->ReadString("Replace"+IntToStr(i), "DataMask", "");
      StringGrid->Cells[6][i+1] = fileOut->ReadString("Replace"+IntToStr(i), "DataFilter", "");
      StringGrid->Cells[7][i+1] = fileOut->ReadString("Replace"+IntToStr(i), "NewDataMask", "");
      StringGrid->Cells[8][i+1] = fileOut->ReadString("Replace"+IntToStr(i), "NewDataValue", "");
    }

    delete fileOut;
  }        
}
//---------------------------------------------------------------------------

void TMainForm::MoveRowsDown(int Index)
{
  StringGrid->RowCount++;
  
  for (int i = StringGrid->RowCount-1; i > Index; i--)
  {
    for (int j = 0; j < StringGrid->ColCount; j++)
    {
      StringGrid->Cells[j][i] = StringGrid->Cells[j][i-1];
    }
  }
}

void TMainForm::MoveRowsUp(int Index)
{
  for (int i = Index; i < StringGrid->RowCount; i++)
  {
    for (int j = 0; j < StringGrid->ColCount; j++)
    {
      StringGrid->Cells[j][i-1] = StringGrid->Cells[j][i];
    }
  }

  StringGrid->RowCount--;
}
//---------------------------------------------------------------------------
void __fastcall TMainForm::Button_RowCloneClick(TObject *Sender)
{
  if (StringGrid->RowCount-1 >= MAX_FILTER_COUNT) return;
  MoveRowsDown(StringGrid->Row);
  RenumberRows();
}
//---------------------------------------------------------------------------

void __fastcall TMainForm::Button_RowDeleteClick(TObject *Sender)
{
  if (StringGrid->RowCount <= 2) return;
  if (Application->MessageBox("Are you sure you want to delete this row?", "Delete confirmation", MB_YESNO | MB_ICONQUESTION) == IDYES)
  {
    MoveRowsUp(StringGrid->Row+1);  
  }
  RenumberRows();
}
//---------------------------------------------------------------------------

void __fastcall TMainForm::Button_InsertClick(TObject *Sender)
{
  if (StringGrid->RowCount-1 >= MAX_FILTER_COUNT) return;
  MoveRowsDown(StringGrid->Row);
  for (int j = 0; j < StringGrid->ColCount; j++)
  {
    StringGrid->Cells[j][StringGrid->Row] = "";
  }
  RenumberRows();
}
//---------------------------------------------------------------------------

void TMainForm::RenumberRows()
{
  for (int i = 1; i < StringGrid->RowCount; i++)
  {
    StringGrid->Cells[0][i] = i;
  }
}

AnsiString NoSpaces(AnsiString s)
{
  AnsiString sRes = "";
  int iLen = s.Length();
  for (int i = 1; i <= iLen; i++)
  {
    if (s[i] != ' ')
      sRes = sRes + s[i];
  }
  return sRes;
}

bool TMainForm::ValidateStringGridContents()
{
  // this is fix for fixed row count in empty grid
  if (IsRowEmpty(1) && StringGrid->RowCount == 2) return 1; // this is empty grid

  for (int i = 1; i < StringGrid->RowCount; i++)
  {
    for (int j = 1; j < StringGrid->ColCount; j++)
    {
      unsigned __int64 iValue;
      char* sEnd = 0;

      iValue = strtoull(AnsiString("0x"+NoSpaces(StringGrid->Cells[j][i])).c_str(), &sEnd, 16);
      if (*sEnd != 0)
      {
        ShowMessage("Check value in row "+IntToStr(i) + " and col " + IntToStr(j));
        return 0;
      }
    }
  }
  return 1;
}

bool TMainForm::IsRowEmpty(int Index)
{
  AnsiString sStr = "";
  for (int j = 1; j < StringGrid->ColCount; j++)
  {
    sStr = sStr + StringGrid->Cells[j][Index];
  }
  return (NoSpaces(sStr).Length() == 0);
}
//---------------------------------------------------------------------------

