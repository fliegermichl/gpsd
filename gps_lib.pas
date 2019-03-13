{ This file was automatically created by Lazarus. Do not edit!
  This source is only used to compile and install the package.
 }

unit gps_lib;

interface

uses
  gps, LazarusPackageIntf;

implementation

procedure Register;
begin
end;

initialization
  RegisterPackage('gps_lib', @Register);
end.
