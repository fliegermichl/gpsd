unit gpstestmainu;

{$mode objfpc}{$H+}

interface

uses
  Classes, SysUtils, FileUtil, Forms, Controls, Graphics, Dialogs, StdCtrls,
  ExtCtrls, gps;

type

  { TForm1 }

  TForm1 = class(TForm)
    Button1: TButton;
    Button2: TButton;
    Memo1: TMemo;
    Timer1: TTimer;
    procedure Button1Click(Sender: TObject);
    procedure Button2Click(Sender: TObject);
    procedure Timer1Timer(Sender: TObject);
  private
    fUpdating : boolean;
    gps_data : gps_data_t;
  public

  end;

var
  Form1: TForm1;

implementation


{$R *.lfm}

{ TForm1 }

procedure TForm1.Button1Click(Sender: TObject);
var
    rc : longint;
begin
 if gps_open('localhost', '2947', @gps_data) <0 then
 begin
   MessageDlg('Yep', mtInformation, [mbOk], 0);
 end;
 gps_stream(@gps_data, WATCH_ENABLE, NIL);
 if gps_waiting(@gps_data, 20000000) then
 begin
  rc := gps_read(@gps_data);
  if (rc >= 0) then
  begin
  end;
 end;
end;

procedure TForm1.Button2Click(Sender: TObject);
begin

end;

procedure TForm1.Timer1Timer(Sender: TObject);
var
    rc : longint;
begin
 if fUpdating then exit;
 fUpdating := True;
 try
   if gps_open('raspberrypi', '2947', @gps_data) <0 then
   begin
     MessageDlg('Yep', mtInformation, [mbOk], 0);
   end;
   gps_stream(@gps_data, WATCH_ENABLE, NIL);
   if gps_waiting(@gps_data, 20000000) then
   begin
    rc := gps_read(@gps_data);
    if (rc >= 0) then
    begin
     Memo1.Lines.Add(Format('%.3f', [gps_data.online]));
    end;
   end;
 finally
   fUpdating := False;
 end;
end;


end.

