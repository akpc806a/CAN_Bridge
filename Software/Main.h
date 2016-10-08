//---------------------------------------------------------------------------

#ifndef MainH
#define MainH
//---------------------------------------------------------------------------
#include <Classes.hpp>
#include <Controls.hpp>
#include <StdCtrls.hpp>
#include <Forms.hpp>

#include "Interface.h"
#include <ComCtrls.hpp>
#include <Grids.hpp>
#include <Dialogs.hpp>
//---------------------------------------------------------------------------
class TMainForm : public TForm
{
__published:	// IDE-managed Components
        TStatusBar *StatusBar1;
        TPageControl *PageControl1;
        TTabSheet *TabSheet_Main;
        TTabSheet *TabSheet2;
        TGroupBox *GroupBox1;
        TButton *Button_Open;
        TComboBox *ComboBox_COMPorts;
        TButton *Button_Read;
        TGroupBox *GroupBox2;
        TLabel *Label1;
        TLabel *Label2;
        TLabel *Label3;
        TComboBox *ComboBox_Baudrate1;
        TEdit *Edit_FilterId1;
        TEdit *Edit_FilterMask1;
        TGroupBox *GroupBox3;
        TLabel *Label4;
        TLabel *Label5;
        TLabel *Label6;
        TComboBox *ComboBox_Baudrate2;
        TEdit *Edit_FilterId2;
        TEdit *Edit_FilterMask2;
        TButton *Button_Write;
        TStringGrid *StringGrid;
        TButton *Button_Load;
        TButton *Button_Save;
        TGroupBox *GroupBox_Grid;
        TButton *Button_RowAdd;
        TButton *Button_RowClone;
        TButton *Button_RowDelete;
        TOpenDialog *OpenDialog;
        TSaveDialog *SaveDialog;
        TButton *Button_Insert;
        void __fastcall FormShow(TObject *Sender);
        void __fastcall ComboBox_COMPortsDropDown(TObject *Sender);
        void __fastcall Button_OpenClick(TObject *Sender);
        void __fastcall Button_ReadClick(TObject *Sender);
        void __fastcall Button_WriteClick(TObject *Sender);
        void __fastcall FormCreate(TObject *Sender);
        void __fastcall Button_RowAddClick(TObject *Sender);
        void __fastcall Button_SaveClick(TObject *Sender);
        void __fastcall Button_LoadClick(TObject *Sender);
        void __fastcall Button_RowCloneClick(TObject *Sender);
        void __fastcall Button_RowDeleteClick(TObject *Sender);
        void __fastcall Button_InsertClick(TObject *Sender);
private:	// User declarations
        TCOMCtrl *InterfaceCtrl;
        void MoveRowsDown(int Index);
        void MoveRowsUp(int Index);
        void RenumberRows();
        bool ValidateStringGridContents();
        bool IsRowEmpty(int Index);
public:		// User declarations
        __fastcall TMainForm(TComponent* Owner);
};
//---------------------------------------------------------------------------
extern PACKAGE TMainForm *MainForm;
//---------------------------------------------------------------------------
#endif
