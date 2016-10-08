object MainForm: TMainForm
  Left = 239
  Top = 142
  Width = 677
  Height = 495
  Caption = 'CAN Bridge Configuration Tool ver 0.1'
  Color = clBtnFace
  Font.Charset = DEFAULT_CHARSET
  Font.Color = clWindowText
  Font.Height = -11
  Font.Name = 'MS Sans Serif'
  Font.Style = []
  OldCreateOrder = False
  OnCreate = FormCreate
  OnShow = FormShow
  PixelsPerInch = 96
  TextHeight = 13
  object StatusBar1: TStatusBar
    Left = 0
    Top = 438
    Width = 661
    Height = 19
    Panels = <>
    SimplePanel = False
  end
  object PageControl1: TPageControl
    Left = 0
    Top = 0
    Width = 661
    Height = 438
    ActivePage = TabSheet_Main
    Align = alClient
    TabIndex = 0
    TabOrder = 1
    object TabSheet_Main: TTabSheet
      Caption = 'Main'
      object GroupBox1: TGroupBox
        Left = 8
        Top = 8
        Width = 233
        Height = 49
        Caption = 'COM port'
        TabOrder = 0
        object Button_Open: TButton
          Left = 150
          Top = 16
          Width = 75
          Height = 25
          Caption = 'Open'
          TabOrder = 0
          OnClick = Button_OpenClick
        end
        object ComboBox_COMPorts: TComboBox
          Left = 8
          Top = 20
          Width = 145
          Height = 21
          ItemHeight = 13
          TabOrder = 1
          Text = 'ComboBox_COMPorts'
          OnDropDown = ComboBox_COMPortsDropDown
        end
      end
      object Button_Read: TButton
        Left = 8
        Top = 64
        Width = 75
        Height = 25
        Caption = 'Read'
        Enabled = False
        TabOrder = 1
        OnClick = Button_ReadClick
      end
      object GroupBox2: TGroupBox
        Left = 8
        Top = 104
        Width = 233
        Height = 129
        Caption = 'CAN 1 (left)'
        TabOrder = 2
        object Label1: TLabel
          Left = 16
          Top = 32
          Width = 57
          Height = 13
          Caption = 'Speed kbps'
        end
        object Label2: TLabel
          Left = 16
          Top = 64
          Width = 36
          Height = 13
          Caption = 'Filter ID'
        end
        object Label3: TLabel
          Left = 16
          Top = 96
          Width = 51
          Height = 13
          Caption = 'Filter Mask'
        end
        object ComboBox_Baudrate1: TComboBox
          Left = 88
          Top = 24
          Width = 121
          Height = 21
          ItemHeight = 13
          ItemIndex = 3
          TabOrder = 0
          Text = '1000'
          Items.Strings = (
            '125'
            '250'
            '500'
            '1000')
        end
        object Edit_FilterId1: TEdit
          Left = 88
          Top = 56
          Width = 121
          Height = 22
          Font.Charset = RUSSIAN_CHARSET
          Font.Color = clWindowText
          Font.Height = -11
          Font.Name = 'Courier New'
          Font.Style = []
          ParentFont = False
          TabOrder = 1
          Text = '00000000'
        end
        object Edit_FilterMask1: TEdit
          Left = 88
          Top = 88
          Width = 121
          Height = 22
          Font.Charset = RUSSIAN_CHARSET
          Font.Color = clWindowText
          Font.Height = -11
          Font.Name = 'Courier New'
          Font.Style = []
          ParentFont = False
          TabOrder = 2
          Text = '00000000'
        end
      end
      object GroupBox3: TGroupBox
        Left = 256
        Top = 104
        Width = 233
        Height = 129
        Caption = 'CAN 2 (right)'
        TabOrder = 3
        object Label4: TLabel
          Left = 16
          Top = 32
          Width = 57
          Height = 13
          Caption = 'Speed kbps'
        end
        object Label5: TLabel
          Left = 16
          Top = 64
          Width = 36
          Height = 13
          Caption = 'Filter ID'
        end
        object Label6: TLabel
          Left = 16
          Top = 96
          Width = 51
          Height = 13
          Caption = 'Filter Mask'
        end
        object ComboBox_Baudrate2: TComboBox
          Left = 88
          Top = 24
          Width = 121
          Height = 21
          ItemHeight = 13
          ItemIndex = 3
          TabOrder = 0
          Text = '1000'
          Items.Strings = (
            '125'
            '250'
            '500'
            '1000')
        end
        object Edit_FilterId2: TEdit
          Left = 88
          Top = 56
          Width = 121
          Height = 22
          Font.Charset = RUSSIAN_CHARSET
          Font.Color = clWindowText
          Font.Height = -11
          Font.Name = 'Courier New'
          Font.Style = []
          ParentFont = False
          TabOrder = 1
          Text = '00000000'
        end
        object Edit_FilterMask2: TEdit
          Left = 88
          Top = 88
          Width = 121
          Height = 22
          Font.Charset = RUSSIAN_CHARSET
          Font.Color = clWindowText
          Font.Height = -11
          Font.Name = 'Courier New'
          Font.Style = []
          ParentFont = False
          TabOrder = 2
          Text = '00000000'
        end
      end
      object Button_Write: TButton
        Left = 88
        Top = 64
        Width = 75
        Height = 25
        Caption = 'Write'
        Enabled = False
        TabOrder = 4
        OnClick = Button_WriteClick
      end
      object Button_Load: TButton
        Left = 8
        Top = 248
        Width = 75
        Height = 25
        Caption = 'Load Config...'
        TabOrder = 5
        OnClick = Button_LoadClick
      end
      object Button_Save: TButton
        Left = 88
        Top = 248
        Width = 75
        Height = 25
        Caption = 'Save Config...'
        TabOrder = 6
        OnClick = Button_SaveClick
      end
    end
    object TabSheet2: TTabSheet
      Caption = 'Replace'
      ImageIndex = 1
      DesignSize = (
        653
        410)
      object StringGrid: TStringGrid
        Left = 0
        Top = 0
        Width = 653
        Height = 353
        Align = alTop
        Anchors = [akLeft, akTop, akRight, akBottom]
        ColCount = 9
        DefaultColWidth = 170
        RowCount = 2
        Font.Charset = RUSSIAN_CHARSET
        Font.Color = clWindowText
        Font.Height = -11
        Font.Name = 'Courier New'
        Font.Style = []
        Options = [goFixedVertLine, goFixedHorzLine, goVertLine, goHorzLine, goRangeSelect, goColSizing, goEditing, goTabs]
        ParentFont = False
        TabOrder = 0
      end
      object GroupBox_Grid: TGroupBox
        Left = 0
        Top = 352
        Width = 353
        Height = 49
        Anchors = [akLeft, akBottom]
        Caption = 'Row actions'
        TabOrder = 1
        object Button_RowAdd: TButton
          Left = 8
          Top = 16
          Width = 75
          Height = 25
          Caption = 'Append'
          TabOrder = 0
          OnClick = Button_RowAddClick
        end
        object Button_RowClone: TButton
          Left = 96
          Top = 16
          Width = 75
          Height = 25
          Caption = 'Clone'
          TabOrder = 1
          OnClick = Button_RowCloneClick
        end
        object Button_RowDelete: TButton
          Left = 272
          Top = 16
          Width = 75
          Height = 25
          Caption = 'Delete'
          TabOrder = 2
          OnClick = Button_RowDeleteClick
        end
        object Button_Insert: TButton
          Left = 184
          Top = 16
          Width = 75
          Height = 25
          Caption = 'Insert'
          TabOrder = 3
          OnClick = Button_InsertClick
        end
      end
    end
  end
  object OpenDialog: TOpenDialog
    DefaultExt = '.txt'
    Filter = 'Text|*.txt|Any|*.*'
    Left = 56
    Top = 312
  end
  object SaveDialog: TSaveDialog
    DefaultExt = 'txt'
    Filter = 'Text|*.txt|Any|*.*'
    Left = 88
    Top = 312
  end
end
