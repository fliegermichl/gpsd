{$packrecords c}
unit gps;
interface
uses ctypes, lcltype;

const
  gpslib = 'libgps.so';

{
  Automatically converted by H2Pas 1.0.0 from gps.h
  The following command line parameters were used:
    gps.h
}
{ gps.h -- interface of the libgps library  }
{
 * This file is Copyright (c) 2010 by the GPSD project
 * BSD terms apply: see the file COPYING in the distribution root for details.
  }
{ Macro for declaring function arguments unused.  }
{
 * 4.1 - Base version for initial JSON protocol (Dec 2009, release 2.90)
 * 4.2 - AIS application IDs split into DAC and FID (July 2010, release 2.95)
 * 5.0 - MAXCHANNELS bumped from 20 to 32 for GLONASS (Mar 2011, release 2.96)
 *       gps_open() becomes reentrant, what gps_open_r() used to be.
 *       gps_poll() removed in favor of gps_read().  The raw hook is gone.
 * 5.1 - GPS_PATH_MAX uses system PATH_MAX; split24 flag added. New
 *       model and serial members in part B of AIS type 24, conforming
 *       with ITU-R 1371-4. New timedrift structure (Nov 2013, release 3.10).
  }
{ bump on incompatible changes  }

const
  GPSD_API_MAJOR_VERSION = 5;
{ bump on compatible changes  }
  GPSD_API_MINOR_VERSION = 1;
{ maximum length of sentence tag name  }
  MAXTAGLEN = 8;
{ must be > 12 GPS + 12 GLONASS + 2 WAAS  }
  MAXCHANNELS = 72;
{ above this number are SBAS satellites  }
  GPS_PRNMAX = 32;
{ max devices per user  }
  MAXUSERDEVS = 4;
{ PATH_MAX needs to be enough for long names like /dev/serial/by-id/...  }

 GPS_PATH_MAX = 4096;
{
 * Someday we may support Windows, under which socket_t is a separate type.
 * In the meantime, having a typedef for this semantic kind is no bad thing,
 * as it makes clearer what some declarations are doing without breaking
 * binary compatibility.
  }

type
  socket_t = longint;
  uint32_t = cardinal;
  time_t = longint;
  uint8_t = byte;
  int16_t = smallint;
  int32_t = longint;
  uint16_t = word;
  int8_t = shortint;
  size_t = csize_t;
type
  timestamp_t = double;
  timespec = longint;
{ Unix time in seconds with fractional part  }
{ mode update not seen yet  }

  Type
    uint64_t = uint64;
    gps_mask_t = uint64_t;

    { is watcher mode on?  }
    { requesting JSON?  }
    { requesting dumping as NMEA?  }
    { requesting raw data?  }
    { requesting report scaling?  }
    { requesting timing info  }
    { requesting split AIS Type 24s  }
    { requesting PPS in NMEA/raw modes  }
    { requested log level of messages  }
    { specific device to watch  }
    { ...if this was passthrough  }
      policy_t = record
          watcher : boolean;
          json : boolean;
          nmea : boolean;
          raw : longint;
          scaled : boolean;
          timing : boolean;
          split24 : boolean;
          pps : boolean;
          loglevel : longint;
          devpath : array[0..(GPS_PATH_MAX)-1] of char;
          remote : array[0..(GPS_PATH_MAX)-1] of char;
        end;

      timedrift_t = record
          real : timespec;
          clock : timespec;
        end;


  { Dilution of precision factors  }
    dop_t = record
        xdop : double;
        ydop : double;
        pdop : double;
        hdop : double;
        vdop : double;
        tdop : double;
        gdop : double;
      end;
  const
    MODE_NOT_SEEN = 0;
  { none  }
    MODE_NO_FIX = 1;
  { good for latitude/longitude  }
    MODE_2D = 2;
  { good for altitude/climb too  }
    MODE_3D = 3;
  { Time of update  }
  { Mode of fix  }
  { Expected time uncertainty  }
  { Latitude in degrees (valid if mode >= 2)  }
  { Latitude position uncertainty, meters  }
  { Longitude in degrees (valid if mode >= 2)  }
  { Longitude position uncertainty, meters  }
  { Altitude in meters (valid if mode == 3)  }
  { Vertical position uncertainty, meters  }
  { Course made good (relative to true north)  }
  { Track uncertainty, degrees  }
  { Speed over ground, meters/sec  }
  { Speed uncertainty, meters/sec  }
  { Vertical speed, meters/sec  }
  { Vertical speed uncertainty  }

  type
    gps_fix_t = record
        time : timestamp_t;
        mode : longint;
        ept : double;
        latitude : double;
        epy : double;
        longitude : double;
        epx : double;
        altitude : double;
        epv : double;
        track : double;
        epd : double;
        speed : double;
        eps : double;
        climb : double;
        epc : double;
      end;

    { unitvector sqrt(x^2 + y^2 +z^2)  }
    { unitvector sqrt(x^2 + y^2 +z^2)  }
    { compass status -- TrueNorth (and any similar) devices only  }
      attitude_t = record
          heading : double;
          pitch : double;
          roll : double;
          yaw : double;
          dip : double;
          mag_len : double;
          mag_x : double;
          mag_y : double;
          mag_z : double;
          acc_len : double;
          acc_x : double;
          acc_y : double;
          acc_z : double;
          gyro_x : double;
          gyro_y : double;
          temp : double;
          depth : double;
          mag_st : char;
          pitch_st : char;
          roll_st : char;
          yaw_st : char;
        end;


    { satellite acquired  }

    const
      SAT_ACQUIRED = $01;
    { code-tracking loop acquired  }
      SAT_CODE_TRACK = $02;
    { carrier-tracking loop acquired  }
      SAT_CARR_TRACK = $04;
    { data-bit synchronization done  }
      SAT_DATA_SYNC = $08;
    { frame synchronization done  }
      SAT_FRAME_SYNC = $10;
    { ephemeris collected  }
      SAT_EPHEMERIS = $20;
    { used for position fix  }
      SAT_FIX_USED = $40;
    { raw measurement data  }
    { meters  }
    { meters  }
    { meters  }
    { meters/sec  }
    { Hz  }
    { sec  }
    { tracking status  }

    type
      rawdata_t = record
          codephase : array[0..(MAXCHANNELS)-1] of double;
          carrierphase : array[0..(MAXCHANNELS)-1] of double;
          pseudorange : array[0..(MAXCHANNELS)-1] of double;
          deltarange : array[0..(MAXCHANNELS)-1] of double;
          doppler : array[0..(MAXCHANNELS)-1] of double;
          mtime : array[0..(MAXCHANNELS)-1] of double;
          satstat : array[0..(MAXCHANNELS)-1] of dword;
        end;

    { external version  }
    { internal revision ID  }
    { API major and minor versions  }
    { could be from a remote device  }
      version_t = record
          release : array[0..63] of char;
          rev : array[0..63] of char;
          proto_major : longint;
          proto_minor : longint;
          remote : array[0..(GPS_PATH_MAX)-1] of char;
        end;


    const
      SEEN_GPS = $01;
      SEEN_RTCM2 = $02;
      SEEN_RTCM3 = $04;
      SEEN_AIS = $08;
    { RS232 link parameters  }
    { 'N', 'O', or 'E'  }
    { refresh cycle time in seconds  }
    { is driver in native mode or not?  }

    type
      devconfig_t = record
          path : array[0..(GPS_PATH_MAX)-1] of char;
          flags : longint;
          driver : array[0..63] of char;
          subtype : array[0..63] of char;
          activated : double;
          baudrate : dword;
          stopbits : dword;
          parity : char;
          cycle : double;
          mincycle : double;
          driver_mode : longint;
        end;

      {
       * GLONASS birds reuse GPS PRNs.
       * it is a GPSD convention to map them to IDs 65..96.
       * (some other programs push them to 33 and above).
        }
      const
        GLONASS_PRN_OFFSET = 64;
      {
       * The structure describing the pseudorange errors (GPGST)
        }

      type
        gst_t = record
            utctime : double;
            rms_deviation : double;
            smajor_deviation : double;
            sminor_deviation : double;
            smajor_orientation : double;
            lat_err_deviation : double;
            lon_err_deviation : double;
            alt_err_deviation : double;
          end;

      {
       * From the RCTM104 2.x standard:
       *
       * "The 30 bit words (as opposed to 32 bit words) coupled with a 50 Hz
       * transmission rate provides a convenient timing capability where the
       * times of word boundaries are a rational multiple of 0.6 seconds."
       *
       * "Each frame is N+2 words long, where N is the number of message data
       * words. For example, a filler message (type 6 or 34) with no message
       * data will have N=0, and will consist only of two header words. The
       * maximum number of data words allowed by the format is 31, so that
       * the longest possible message will have a total of 33 words."
        }

      const
        RTCM2_WORDS_MAX = 33;
      { max correction count in type 1 or 9  }
        MAXCORRECTIONS = 18;
      { maximum stations in almanac, type 5  }
        MAXSTATIONS = 10;
      { RTCM104 doesn't specify this, so give it the largest reasonable value  }
        MAXHEALTH = RTCM2_WORDS_MAX-2;
      {
       * A nominally 30-bit word (24 bits of data, 6 bits of parity)
       * used both in the GPS downlink protocol described in IS-GPS-200
       * and in the format for DGPS corrections used in RTCM-104v2.
        }
      {@unsignedintegraltype@ }
      type
        isgps30bits_t = uint32_t;
      {
       * Values for "system" fields.  Note, the encoding logic is senstive to the
       * actual values of these; it's not sufficient that they're distinct.
        }

      const
        NAVSYSTEM_GPS = 0;
        NAVSYSTEM_GLONASS = 1;
        NAVSYSTEM_GALILEO = 2;
        NAVSYSTEM_UNKNOWN = 3;
        SENSE_INVALID = 0;
        SENSE_GLOBAL = 1;
        SENSE_LOCAL = 2;
      { Radiobeacon operation normal  }
        HEALTH_NORMAL = 0;
      { No integrity monitor operating  }
        HEALTH_UNMONITORED = 1;
      { No information available  }
        HEALTH_NOINFO = 2;
      { Do not use this radiobeacon  }
        HEALTH_DONOTUSE = 3;
      { not reported  }
        SNR_BAD = -(1);
      { header contents  }
      { RTCM message type  }
      { length (words)  }
      { time within hour: GPS time, no leap secs  }
      { reference station ID  }
      { message sequence number (modulo 8)  }
      { station health  }
      { message data in decoded form  }
      {    union  }
      { data from messages 1 & 9  }
      { satellite ID  }
      { user diff. range error  }
      { issue of data  }
      { range error  }
      { range error rate  }
      { data for type 3 messages  }
      { is message well-formed?  }
      { data from type 4 messages  }
      { is message well-formed?  }
      { data from type 5 messages  }
      { satellite ID  }
      { issue of data  }
      { is satellite healthy?  }
      { signal-to-noise ratio, dB  }
      { health enabled  }
      { new data?  }
      { line-of-sight warning  }
      { time to unhealth, seconds  }
      { data from type 7 messages  }
      { location  }
      { range in km  }
      { broadcast freq  }
      { station health  }
      { of the transmitter  }
      { of station transmissions  }
      { data for type 13 messages  }
      { expect a text message  }
      { station range altered?  }
      { station longitude/latitude  }
      { transmission range in km  }
      { data from type 14 messages  }
      { GPS week (0-1023)  }
      { Hour in week (0-167)  }
      { Leap seconds (0-63)  }
      { data from message type 31  }
      { satellite ID  }
      { user diff. range error  }
      { issue of data  }
      { ephemeris change bit  }
      { range error  }
      { range error rate  }
      { data from type 16 messages  }
      { data from messages of unknown type  }
      {    ; }

      type
        rtcm2_t = record
            _type : dword;
            length : dword;
            zcount : double;
            refstaid : dword;
            seqnum : dword;
            stathlth : dword;
        case longint of
         1 : (
            gps_ranges : record
                nentries : dword;
                sat : array[0..(MAXCORRECTIONS)-1] of record
                    ident : dword;
                    udre : dword;
                    iod : dword;
                    prc : double;
                    rrc : double;
                  end;
              end;
              );
         2 : (
            ecef : record
                valid : bool;
                x : double;
                y : double;
                z : double;
              end;
              );
         3 : (
            reference : record
                valid : bool;
                system : longint;
                sense : longint;
                datum : array[0..5] of char;
                dx : double;
                dy : double;
                dz : double;
              end;
            );
         4 : (
            conhealth : record
                nentries : dword;
                sat : array[0..(MAXHEALTH)-1] of record
                    ident : dword;
                    iodl : boolean;
                    health : dword;
                    snr : longint;
                    health_en : boolean;
                    new_data : boolean;
                    los_warning : boolean;
                    tou : dword;
                  end;
              end;
            );
         5 : (
            almanac : record
                nentries : dword;
                station : array[0..(MAXSTATIONS)-1] of record
                    latitude : double;
                    longitude : double;
                    range : dword;
                    frequency : double;
                    health : dword;
                    station_id : dword;
                    bitrate : dword;
                  end;
              end;
            );
         6 : (
            xmitter : record
                status : boolean;
                rangeflag : boolean;
                lat : double;
                lon : double;
                range : dword;
              end;
              );
         7 : (
            gpstime : record
                week : dword;
                hour : dword;
                leapsecs : dword;
              end;
            );
         8 : (
            glonass_ranges : record
                nentries : dword;
                sat : array[0..(MAXCORRECTIONS)-1] of record
                    ident : dword;
                    udre : dword;
                    tod : dword;
                    change : boolean;
                    prc : double;
                    rrc : double;
                  end;
              end;
            );
         9 : (
            message : array[0..((RTCM2_WORDS_MAX-2)*(sizeof(isgps30bits_t)))-1] of char;
            );
         10 : (
            words : array[0..(RTCM2_WORDS_MAX-2)-1] of isgps30bits_t;
            );
          end;

      { RTCM3 report structures begin here  }

      const
        RTCM3_MAX_SATELLITES = 64;
        RTCM3_MAX_DESCRIPTOR = 31;
        RTCM3_MAX_ANNOUNCEMENTS = 32;
      { header data from 1001, 1002, 1003, 1004  }
      { Used for both GPS and GLONASS, but their timebases differ  }
      { Reference Station ID  }
      { GPS Epoch Time (TOW) in ms,
      				   or GLONASS Epoch Time in ms  }
      { Synchronous GNSS Message Flag  }
      { # Satellite Signals Processed  }
      { Divergence-free Smoothing Indicator  }
      { Smoothing Interval  }

      type
        rtcm3_rtk_hdr = record
            station_id : dword;
            tow : time_t;
            sync : boolean;
            satcount : word;
            smoothing : boolean;
            interval : dword;
          end;

      { Indicator  }
      { Satellite Frequency Channel Number
      				   (GLONASS only)  }
      { Pseudorange  }
      { PhaseRange – Pseudorange in meters  }
      { Lock time Indicator  }
        rtcm3_basic_rtk = record
            indicator : byte;
            channel : dword;
            pseudorange : double;
            rangediff : double;
            locktime : byte;
          end;

      { Indicator  }
      { Satellite Frequency Channel Number
      				   (GLONASS only)  }
      { Pseudorange  }
      { PhaseRange – L1 Pseudorange  }
      { Lock time Indicator  }
      { Integer Pseudorange
      					   Modulus Ambiguity  }
      { Carrier-to-Noise Ratio  }
        rtcm3_extended_rtk = record
            indicator : byte;
            channel : dword;
            pseudorange : double;
            rangediff : double;
            locktime : byte;
            ambiguity : byte;
            CNR : double;
          end;

      { Network ID  }
      { Subnetwork ID  }
      { GPS Epoch Time (TOW) in ms  }
      { GPS Multiple Message Indicator  }
      { Master Reference Station ID  }
      { Auxilary Reference Station ID  }
      { count of GPS satellites  }
        rtcm3_network_rtk_header = record
            network_id : dword;
            subnetwork_id : dword;
            time : time_t;
            multimesg : boolean;
            master_id : dword;
            aux_id : dword;
            satcount : byte;
          end;

      { satellite ID  }
      { Geometric Carrier Phase
      				   Correction Difference (1016, 1017)  }
      { GPS IODE (1016, 1017)  }
      { Ionospheric Carrier Phase
      				   Correction Difference (1015, 1017)  }
        rtcm3_correction_diff = record
            ident : byte;
            ambiguity : (reserved,correct,widelane,uncertain
              );
            nonsync : byte;
            geometric_diff : double;
            iode : byte;
            ionospheric_diff : double;
          end;

      { header contents  }
      { RTCM 3.x message type  }
      { payload length, inclusive of checksum  }
      { 1001-1013 were present in the 3.0 version  }
      { Satellite ID  }
      { Satellite ID  }
      { Satellite ID  }
      { Satellite ID  }
      { Reference Station ID  }
      { Which system is it?  }
      { Reference-station indicator  }
      { Single Receiver Oscillator  }
      { ECEF antenna location  }
      { Reference Station ID  }
      { Which system is it?  }
      { Reference-station indicator  }
      { Single Receiver Oscillator  }
      { ECEF antenna location  }
      { Antenna height  }
      { Reference Station ID  }
      { Description string  }
      { Reference Station ID  }
      { Description string  }
      { Serial # string  }
      { Satellite ID  }
      { Satellite ID  }
      { Satellite ID  }
      { Satellite ID  }
      { Reference Station ID  }
      { Modified Julian Day (MJD) Number  }
      { Seconds of Day (UTC)  }
      { Leap Seconds, GPS-UTC  }
      { Count of announcements to follow  }
      { message type ID  }
      { interval in 0.1sec units  }
      { 1014-1017 were added in the 3.1 version  }
      { Network ID  }
      { Subnetwork ID  }
      { # auxiliary stations transmitted  }
      { Master Reference Station ID  }
      { Auxilary Reference Station ID  }
      { Aux-master location delta  }
      { 1018-1029 were in the 3.0 version  }
      { Satellite ID  }
      { GPS Week Number  }
      { GPS SV ACCURACY  }
      { ephemeris fields, not scaled  }
      { Satellite ID  }
      { Satellite Frequency Channel Number  }
      { ephemeris fields, not scaled  }
      { Reference Station ID  }
      { Modified Julian Day (MJD) Number  }
      { Seconds of Day (UTC)  }
      { # chars to follow  }
      { # Unicode units in text  }
      { Reference Station ID  }
      { Description string  }
      { Serial # string  }
      { Receiver string  }
      { Firmware string  }
      { Max RTCM3 msg length is 1023 bytes  }
        rtcm3_t = record
            _type : dword;
            length : dword;
            rtcmtypes : record
                case longint of
                  0 : ( rtcm3_1001 : record
                      header : rtcm3_rtk_hdr;
                      rtk_data : array[0..(RTCM3_MAX_SATELLITES)-1] of record
                          ident : dword;
                          L1 : rtcm3_basic_rtk;
                        end;
                    end );
                  1 : ( rtcm3_1002 : record
                      header : rtcm3_rtk_hdr;
                      rtk_data : array[0..(RTCM3_MAX_SATELLITES)-1] of record
                          ident : dword;
                          L1 : rtcm3_extended_rtk;
                        end;
                    end );
                  2 : ( rtcm3_1003 : record
                      header : rtcm3_rtk_hdr;
                      rtk_data : array[0..(RTCM3_MAX_SATELLITES)-1] of record
                          ident : dword;
                          L1 : rtcm3_basic_rtk;
                          L2 : rtcm3_basic_rtk;
                        end;
                    end );
                  3 : ( rtcm3_1004 : record
                      header : rtcm3_rtk_hdr;
                      rtk_data : array[0..(RTCM3_MAX_SATELLITES)-1] of record
                          ident : dword;
                          L1 : rtcm3_extended_rtk;
                          L2 : rtcm3_extended_rtk;
                        end;
                    end );
                  4 : ( rtcm3_1005 : record
                      station_id : dword;
                      system : longint;
                      reference_station : boolean;
                      single_receiver : boolean;
                      ecef_x : double;
                      ecef_y : double;
                      ecef_z : double;
                    end );
                  5 : ( rtcm3_1006 : record
                      station_id : dword;
                      system : longint;
                      reference_station : boolean;
                      single_receiver : boolean;
                      ecef_x : double;
                      ecef_y : double;
                      ecef_z : double;
                      height : double;
                    end );
                  6 : ( rtcm3_1007 : record
                      station_id : dword;
                      descriptor : array[0..(RTCM3_MAX_DESCRIPTOR+1)-1] of char;
                      setup_id : dword;
                    end );
                  7 : ( rtcm3_1008 : record
                      station_id : dword;
                      descriptor : array[0..(RTCM3_MAX_DESCRIPTOR+1)-1] of char;
                      setup_id : dword;
                      serial : array[0..(RTCM3_MAX_DESCRIPTOR+1)-1] of char;
                    end );
                  8 : ( rtcm3_1009 : record
                      header : rtcm3_rtk_hdr;
                      rtk_data : array[0..(RTCM3_MAX_SATELLITES)-1] of record
                          ident : dword;
                          L1 : rtcm3_basic_rtk;
                        end;
                    end );
                  9 : ( rtcm3_1010 : record
                      header : rtcm3_rtk_hdr;
                      rtk_data : array[0..(RTCM3_MAX_SATELLITES)-1] of record
                          ident : dword;
                          L1 : rtcm3_extended_rtk;
                        end;
                    end );
                  10 : ( rtcm3_1011 : record
                      header : rtcm3_rtk_hdr;
                      rtk_data : array[0..(RTCM3_MAX_SATELLITES)-1] of record
                          ident : dword;
                          L1 : rtcm3_extended_rtk;
                          L2 : rtcm3_extended_rtk;
                        end;
                    end );
                  11 : ( rtcm3_1012 : record
                      header : rtcm3_rtk_hdr;
                      rtk_data : array[0..(RTCM3_MAX_SATELLITES)-1] of record
                          ident : dword;
                          L1 : rtcm3_extended_rtk;
                          L2 : rtcm3_extended_rtk;
                        end;
                    end );
                  12 : ( rtcm3_1013 : record
                      station_id : dword;
                      mjd : word;
                      sod : dword;
                      leapsecs : byte;
                      ncount : byte;
                      announcements : array[0..(RTCM3_MAX_ANNOUNCEMENTS)-1] of record
                          id : word;
                          sync : boolean;
                          interval : word;
                        end;
                    end );
                  13 : ( rtcm3_1014 : record
                      network_id : dword;
                      subnetwork_id : dword;
                      stationcount : dword;
                      master_id : dword;
                      aux_id : dword;
                      d_lat : double;
                      d_lon : double;
                      d_alt : double;
                    end );
                  14 : ( rtcm3_1015 : record
                      header : rtcm3_network_rtk_header;
                      corrections : array[0..(RTCM3_MAX_SATELLITES)-1] of rtcm3_correction_diff;
                    end );
                  15 : ( rtcm3_1016 : record
                      header : rtcm3_network_rtk_header;
                      corrections : array[0..(RTCM3_MAX_SATELLITES)-1] of rtcm3_correction_diff;
                    end );
                  16 : ( rtcm3_1017 : record
                      header : rtcm3_network_rtk_header;
                      corrections : array[0..(RTCM3_MAX_SATELLITES)-1] of rtcm3_correction_diff;
                    end );
                  17 : ( rtcm3_1019 : record
                      ident : dword;
                      week : dword;
                      sv_accuracy : byte;
                      code : (reserved_code,p,ca,l2c);
                      idot : double;
                      iode : byte;
                      t_sub_oc : dword;
                      a_sub_f2 : longint;
                      a_sub_f1 : longint;
                      a_sub_f0 : longint;
                      iodc : dword;
                      C_sub_rs : longint;
                      delta_sub_n : longint;
                      M_sub_0 : longint;
                      C_sub_uc : longint;
                      e : dword;
                      C_sub_us : longint;
                      sqrt_sub_A : dword;
                      t_sub_oe : dword;
                      C_sub_ic : longint;
                      OMEGA_sub_0 : longint;
                      C_sub_is : longint;
                      i_sub_0 : longint;
                      C_sub_rc : longint;
                      argument_of_perigee : longint;
                      omegadot : longint;
                      t_sub_GD : longint;
                      sv_health : byte;
                      p_data : boolean;
                      fit_interval : boolean;
                    end );
                  18 : ( rtcm3_1020 : record
                      ident : dword;
                      channel : word;
                      C_sub_n : boolean;
                      health_avAilability_indicator : boolean;
                      P1 : byte;
                      t_sub_k : word;
                      msb_of_B_sub_n : boolean;
                      P2 : boolean;
                      t_sub_b : boolean;
                      x_sub_n_t_of_t_sub_b_prime : longint;
                      x_sub_n_t_of_t_sub_b : longint;
                      x_sub_n_t_of_t_sub_b_prime_prime : longint;
                      y_sub_n_t_of_t_sub_b_prime : longint;
                      y_sub_n_t_of_t_sub_b : longint;
                      y_sub_n_t_of_t_sub_b_prime_prime : longint;
                      z_sub_n_t_of_t_sub_b_prime : longint;
                      z_sub_n_t_of_t_sub_b : longint;
                      z_sub_n_t_of_t_sub_b_prime_prime : longint;
                      P3 : boolean;
                      gamma_sub_n_of_t_sub_b : longint;
                      MP : byte;
                      Ml_n : boolean;
                      tau_n_of_t_sub_b : longint;
                      M_delta_tau_sub_n : longint;
                      E_sub_n : dword;
                      MP4 : boolean;
                      MF_sub_T : byte;
                      MN_sub_T : byte;
                      MM : byte;
                      additioinal_data_availability : boolean;
                      N_sup_A : dword;
                      tau_sub_c : dword;
                      M_N_sub_4 : dword;
                      M_tau_sub_GPS : longint;
                      M_l_sub_n : boolean;
                    end );
                  19 : ( rtcm3_1029 : record
                      station_id : dword;
                      mjd : word;
                      sod : dword;
                      len : size_t;
                      unicode_units : size_t;
                      text : array[0..127] of byte;
                    end );
                  20 : ( rtcm3_1033 : record
                      station_id : dword;
                      descriptor : array[0..(RTCM3_MAX_DESCRIPTOR+1)-1] of char;
                      setup_id : dword;
                      serial : array[0..(RTCM3_MAX_DESCRIPTOR+1)-1] of char;
                      receiver : array[0..(RTCM3_MAX_DESCRIPTOR+1)-1] of char;
                      firmware : array[0..(RTCM3_MAX_DESCRIPTOR+1)-1] of char;
                    end );
                  21 : ( data : array[0..1023] of char );
                end;
          end;

      { RTCM3 scaling constants  }
      { 1004, DF014 }

      const
        GPS_AMBIGUITY_MODULUS = 299792.458;
      { 1012, DF044  }
        GLONASS_AMBIGUITY_MODULUS = 599584.916;
      { 1013, DF047  }
        MESSAGE_INTERVAL_UNITS = 0.1;
      {
       * Raw IS_GPS subframe data
        }
      { The almanac is a subset of the clock and ephemeris data, with reduced
       * precision. See IS-GPS-200E, Table 20-VI   }
      { The satellite this refers to  }
      { toa, almanac reference time, 8 bits unsigned, seconds  }
      { SV health data, 8 bit unsigned bit map  }
      { deltai, correction to inclination, 16 bits signed, semi-circles  }
      { M0, Mean Anomaly at Reference Time, 24 bits signed, semi-circles  }
      { Omega0, Longitude of Ascending Node of Orbit Plane at Weekly Epoch,
           * 24 bits signed, semi-circles  }
      { omega, Argument of Perigee, 24 bits signed, semi-circles  }
      { af0, SV clock correction constant term
           * 11 bits signed, seconds  }
      { af1, SV clock correction first order term
           * 11 bits signed, seconds/second  }
      { eccentricity, 16 bits, unsigned, dimensionless  }
      { sqrt A, Square Root of the Semi-Major Axis
           * 24 bits unsigned, square_root(meters)  }
      { Omega dot, Rate of Right Ascension, 16 bits signed, semi-circles/sec  }

      type
        almanac_t = record
            sv : uint8_t;
            toa : uint8_t;
            l_toa : longint;
            svh : uint8_t;
            deltai : int16_t;
            d_deltai : double;
            M0 : int32_t;
            d_M0 : double;
            Omega0 : int32_t;
            d_Omega0 : double;
            omega : int32_t;
            d_omega : double;
            af0 : int16_t;
            d_af0 : double;
            af1 : int16_t;
            d_af1 : double;
            e : uint16_t;
            d_eccentricity : double;
            sqrtA : uint32_t;
            d_sqrtA : double;
            Omegad : int16_t;
            d_Omegad : double;
          end;

      { subframe number, 3 bits, unsigned, 1 to 5  }
      { data_id, denotes the NAV data structure of D(t), 2 bits, in
           * IS-GPS-200E always == 0x1  }
      { SV/page id used for subframes 4 & 5, 6 bits  }
      { tSVID, SV ID of the sat that transmitted this frame, 6 bits unsigned  }
      { TOW, Time of Week of NEXT message, 17 bits unsigned, scale 6, seconds  }
      { integrity, URA bounds flag, 1 bit  }
      { alert, alert flag, SV URA and/or the SV User Differential Range
           * Accuracy (UDRA) may be worse than indicated, 1 bit  }
      { antispoof, A-S mode is ON in that SV, 1 bit  }
      {    union  }
      { subframe 1, part of ephemeris, see IS-GPS-200E, Table 20-II
      	 * and Table 20-I  }
      { WN, Week Number, 10 bits unsigned, scale 1, weeks  }
      { IODC, Issue of Data, Clock, 10 bits, unsigned,
      	     * issued in 8 data ranges at the same time  }
      { toc, clock data reference time, 16 bits, unsigned, seconds
      	     * scale 2**4, issued in 8 data ranges at the same time  }
      { l2, code on L2, 2 bits, bit map  }
      { l2p, L2 P data flag, 1 bit  }
      { ura, SV accuracy, 4 bits unsigned index  }
      { hlth, SV health, 6 bits unsigned bitmap  }
      { af0, SV clock correction constant term
      	     * 22 bits signed, scale 2**-31, seconds  }
      { af1, SV clock correction first order term
      	     * 22 bits signed, scale 2**-43, seconds/second  }
      { af2, SV clock correction second order term
      	     * 8 bits signed, scale 2**-55, seconds/second**2  }
      { Tgd,  L1-L2 correction term, 8 bits signed,  scale 2**-31,
      	     * seconds  }
      { subframe 2, part of ephemeris, see IS-GPS-200E, Table 20-II
      	 * and Table 20-III  }
      { Issue of Data (Ephemeris),
      	     * equal to the 8 LSBs of the 10 bit IODC of the same data set  }
      { Age of Data Offset for the NMCT, 6 bits, scale 900,
      	     * ignore if all ones, seconds  }
      { fit, FIT interval flag, indicates a fit interval greater than
      	     * 4 hour, 1 bit  }
      { toe, Reference Time Ephemeris, 16 bits unsigned, scale 2**4,
      	     * seconds  }
      { Crs, Amplitude of the Sine Harmonic Correction Term to the
      	     * Orbit Radius, 16 bits, scale 2**-5, signed, meters  }
      { Cus, Amplitude of the Sine Harmonic Correction Term to the
      	     * Argument of Latitude, 16 bits, signed, scale 2**-29, radians  }
      { Cuc, Amplitude of the Cosine Harmonic Correction Term to the
      	     * Argument of Latitude, 16 bits, signed, scale 2**-29, radians  }
      { deltan, Mean Motion Difference From Computed Value
      	     * Mean Motion Difference From Computed Value
      	     * 16 bits, signed, scale 2**-43, semi-circles/sec  }
      { M0, Mean Anomaly at Reference Time, 32 bits signed,
      	     * scale 2**-31, semi-circles  }
      { eccentricity, 32 bits, unsigned, scale 2**-33, dimensionless  }
      { sqrt A, Square Root of the Semi-Major Axis
      	     * 32 bits unsigned, scale 2**-19, square_root(meters)  }
      { subframe 3, part of ephemeris, see IS-GPS-200E, Table 20-II,
      	 * Table 20-III  }
      { Issue of Data (Ephemeris), 8 bits, unsigned
      	     * equal to the 8 LSBs of the 10 bit IODC of the same data set  }
      { Rate of Inclination Angle, 14 bits signed, scale2**-43,
      	     * semi-circles/sec  }
      { Cic, Amplitude of the Cosine Harmonic Correction Term to the
      	     * Angle of Inclination, 16 bits signed, scale 2**-29, radians }
      { Cis, Amplitude of the Sine Harmonic Correction Term to the
      	     * Angle of Inclination, 16 bits, unsigned, scale 2**-29, radians  }
      { Crc, Amplitude of the Cosine Harmonic Correction Term to the
      	     * Orbit Radius, 16 bits signed, scale 2**-5, meters  }
      { i0, Inclination Angle at Reference Time, 32 bits, signed,
      	     * scale 2**-31, semi-circles  }
      { Omega0, Longitude of Ascending Node of Orbit Plane at Weekly
      	     * Epoch, 32 bits signed, semi-circles  }
      { omega, Argument of Perigee, 32 bits signed, scale 2**-31,
      	     * semi-circles  }
      { Omega dot, Rate of Right Ascension, 24 bits signed,
      	     * scale 2**-43, semi-circles/sec  }
      { subframe 4, page 13  }
      { mapping ord ERD# to SV # is non trivial
      	     * leave it alone.  See IS-GPS-200E Section 20.3.3.5.1.9  }
      { Estimated Range Deviation, 6 bits signed, meters  }
      { ai, Availability Indicator, 2bits, bit map  }
      { subframe 4, page 17, system message, 23 chars, plus nul  }
      { subframe 4, page 18  }
      { ionospheric and UTC data  }
      { A0, Bias coefficient of GPS time scale relative to UTC time
      	     * scale, 32 bits signed, scale 2**-30, seconds  }
      { A1, Drift coefficient of GPS time scale relative to UTC time
      	     * scale, 24 bits signed, scale 2**-50, seconds/second  }
      { alphaX, the four coefficients of a cubic equation representing
      	     * the amplitude of the vertical delay  }
      { alpha0, 8 bits signed, scale w**-30, seconds  }
      { alpha1, 8 bits signed, scale w**-27, seconds/semi-circle  }
      { alpha2, 8 bits signed, scale w**-24, seconds/semi-circle**2  }
      { alpha3, 8 bits signed, scale w**-24, seconds/semi-circle**3  }
      { betaX, the four coefficients of a cubic equation representing
      	     * the period of the model  }
      { beta0, 8 bits signed, scale w**11, seconds  }
      { beta1, 8 bits signed, scale w**14, seconds/semi-circle  }
      { beta2, 8 bits signed, scale w**16, seconds/semi-circle**2  }
      { beta3, 8 bits signed, scale w**16, seconds/semi-circle**3  }
      { leap (delta t ls), current leap second, 8 bits signed,
      	     * scale 1, seconds  }
      { lsf (delta t lsf), future leap second, 8 bits signed,
      	     * scale 1, seconds  }
      { tot, reference time for UTC data,
      	     * 8 bits unsigned, scale 2**12, seconds  }
      { WNt, UTC reference week number, 8 bits unsigned, scale 1,
      	     * weeks  }
      { WNlsf, Leap second reference Week Number,
      	     * 8 bits unsigned, scale 1, weeks  }
      { DN, Leap second reference Day Number , 8 bits unsigned,
      	     * scale 1, days  }
      { subframe 4, page 25  }
      { svf, A-S status and the configuration code of each SV
      	     * 4 bits unsigned, bitmap  }
      { svh, SV health data for SV 25 through 32
      	     * 6 bits unsigned bitmap  }
      { toa, Almanac reference Time, 8 bits unsigned, scale 2**12,
      	     * seconds  }
      { WNa, Week Number almanac, 8 bits, scale 2, GPS Week
      	     * Number % 256  }
      { sv, SV health status, 6 bits, bitmap  }
      {    ; }
        subframe_t = record
            subframe_num : uint8_t;
            data_id : uint8_t;
            pageid : uint8_t;
            tSVID : uint8_t;
            TOW17 : uint32_t;
            l_TOW17 : longint;
            integrity : boolean;
            alert : boolean;
            antispoof : boolean;
            is_almanac : longint;
            sub1 : record
                WN : uint16_t;
                IODC : uint16_t;
                toc : uint16_t;
                l_toc : longint;
                l2 : uint8_t;
                l2p : uint8_t;
                ura : dword;
                hlth : dword;
                af0 : int32_t;
                d_af0 : double;
                af1 : int16_t;
                d_af1 : double;
                af2 : int8_t;
                d_af2 : double;
                Tgd : int8_t;
                d_Tgd : double;
              end;
            sub2 : record
                IODE : uint8_t;
                AODO : uint8_t;
                u_AODO : uint16_t;
                fit : uint8_t;
                toe : uint16_t;
                l_toe : longint;
                Crs : int16_t;
                d_Crs : double;
                Cus : int16_t;
                d_Cus : double;
                Cuc : int16_t;
                d_Cuc : double;
                deltan : int16_t;
                d_deltan : double;
                M0 : int32_t;
                d_M0 : double;
                e : uint32_t;
                d_eccentricity : double;
                sqrtA : uint32_t;
                d_sqrtA : double;
              end;
            sub3 : record
                IODE : uint8_t;
                IDOT : int16_t;
                d_IDOT : double;
                Cic : int16_t;
                d_Cic : double;
                Cis : int16_t;
                d_Cis : double;
                Crc : int16_t;
                d_Crc : double;
                i0 : int32_t;
                d_i0 : double;
                Omega0 : int32_t;
                d_Omega0 : double;
                omega : int32_t;
                d_omega : double;
                Omegad : int32_t;
                d_Omegad : double;
              end;
            sub4 : record
                almanac : almanac_t;
              end;
            sub4_13 : record
                ERD : array[0..32] of char;
                ai : byte;
              end;
            sub4_17 : record
                str : array[0..23] of char;
              end;
            sub4_18 : record
                A0 : int32_t;
                d_A0 : double;
                A1 : int32_t;
                d_A1 : double;
                alpha0 : int8_t;
                d_alpha0 : double;
                alpha1 : int8_t;
                d_alpha1 : double;
                alpha2 : int8_t;
                d_alpha2 : double;
                alpha3 : int8_t;
                d_alpha3 : double;
                beta0 : int8_t;
                d_beta0 : double;
                beta1 : int8_t;
                d_beta1 : double;
                beta2 : int8_t;
                d_beta2 : double;
                beta3 : int8_t;
                d_beta3 : double;
                leap : int8_t;
                lsf : int8_t;
                tot : uint8_t;
                d_tot : double;
                WNt : uint8_t;
                WNlsf : uint8_t;
                DN : uint8_t;
              end;
            sub4_25 : record
                svf : array[0..32] of byte;
                svhx : array[0..7] of uint8_t;
              end;
            sub5 : record
                almanac : almanac_t;
              end;
            sub5_25 : record
                toa : uint8_t;
                l_toa : longint;
                WNa : uint8_t;
                sv : array[0..24] of uint8_t;
              end;
          end;

        { N/A values and scaling constant for 25/24 bit lon/lat pairs  }
        const
          AIS_LON3_NOT_AVAILABLE = 181000;
          AIS_LAT3_NOT_AVAILABLE = 91000;
          AIS_LATLON3_DIV = 60000.0;
        { N/A values and scaling constant for 28/27 bit lon/lat pairs  }
          AIS_LON4_NOT_AVAILABLE = 1810000;
          AIS_LAT4_NOT_AVAILABLE = 910000;
          AIS_LATLON4_DIV = 600000.0;
        { Message Linkage ID  }
        { Sender Class  }
        { Route Type  }
        { Start month  }
        { Start day  }
        { Start hour  }
        { Start minute  }
        { Duration  }
        { Waypoint count  }
        { Longitude  }
        { Latitude  }

        type
          route_info = record
              linkage : dword;
              sender : dword;
              rtype : dword;
              month : dword;
              day : dword;
              hour : dword;
              minute : dword;
              duration : dword;
              waycount : longint;
              waypoints : array[0..15] of record
                  lon : longint;
                  lat : longint;
                end;
            end;


        const
          AIS_TURN_HARD_LEFT = -(127);
          AIS_TURN_HARD_RIGHT = 127;
          AIS_TURN_NOT_AVAILABLE = 128;
          AIS_SPEED_NOT_AVAILABLE = 1023;
        { >= 102.2 knots  }
          AIS_SPEED_FAST_MOVER = 1022;
          AIS_LATLON_DIV = 600000.0;
          AIS_LON_NOT_AVAILABLE = $6791AC0;
          AIS_LAT_NOT_AVAILABLE = $3412140;
          AIS_COURSE_NOT_AVAILABLE = 3600;
          AIS_HEADING_NOT_AVAILABLE = 511;
          AIS_SEC_NOT_AVAILABLE = 60;
          AIS_SEC_MANUAL = 61;
          AIS_SEC_ESTIMATED = 62;
          AIS_SEC_INOPERATIVE = 63;
          AIS_YEAR_NOT_AVAILABLE = 0;
          AIS_MONTH_NOT_AVAILABLE = 0;
          AIS_DAY_NOT_AVAILABLE = 0;
          AIS_HOUR_NOT_AVAILABLE = 24;
          AIS_MINUTE_NOT_AVAILABLE = 60;
          AIS_SECOND_NOT_AVAILABLE = 60;
          AIS_SHIPNAME_MAXLEN = 20;
        { 920 bits  }
          AIS_TYPE6_BINARY_MAX = 920;
          DAC200FID22_STATUS_OPERATIONAL = 0;
          DAC200FID22_STATUS_LIMITED = 1;
          DAC200FID22_STATUS_OUT_OF_ORDER = 2;
          DAC200FID22_STATUS_NOT_AVAILABLE = 0;
          DAC200FID55_COUNT_NOT_AVAILABLE = 255;
          DAC1FID21_VISIBILITY_NOT_AVAILABLE = 127;
          DAC1FID21_VISIBILITY_SCALE = 10.0;
          DAC1FID21_WSPEED_NOT_AVAILABLE = 127;
          DAC1FID21_WDIR_NOT_AVAILABLE = 360;
          DAC1FID21_NONWMO_PRESSURE_NOT_AVAILABLE = 403;
        { > 1200hPa  }
          DAC1FID21_NONWMO_PRESSURE_HIGH = 402;
        { N/A  }
          DAC1FID21_NONWMO_PRESSURE_OFFSET = 400;
          DAC1FID21_AIRTEMP_NOT_AVAILABLE = -(1024);
          DAC1FID21_AIRTEMP_SCALE = 10.0;
          DAC1FID21_WATERTEMP_NOT_AVAILABLE = 501;
          DAC1FID21_WATERTEMP_SCALE = 10.0;
          DAC1FID21_WAVEPERIOD_NOT_AVAILABLE = 63;
          DAC1FID21_WAVEDIR_NOT_AVAILABLE = 360;
          DAC1FID21_SOG_NOT_AVAILABLE = 31;
          DAC1FID21_SOG_HIGH_SPEED = 30;
          DAC1FID21_SOG_SCALE = 2.0;
          DAC1FID21_HDG_NOT_AVAILABLE = 127;
          DAC1FID21_HDG_SCALE = 5.0;
          DAC1FID21_WMO_PRESSURE_SCALE = 10;
          DAC1FID21_WMO_PRESSURE_OFFSET = 90.0;
          DAC1FID21_PDELTA_SCALE = 10;
          DAC1FID21_PDELTA_OFFSET = 50.0;
          DAC1FID21_TWINDDIR_NOT_AVAILABLE = 127;
          DAC1FID21_TWINDSPEED_SCALE = 2;
          DAC1FID21_RWINDSPEED_NOT_AVAILABLE = 255;
          DAC1FID21_RWINDDIR_NOT_AVAILABLE = 127;
          DAC1FID21_RWINDSPEED_SCALE = 2;
          DAC1FID21_MGUSTSPEED_SCALE = 2;
          DAC1FID21_MGUSTSPEED_NOT_AVAILABLE = 255;
          DAC1FID21_MGUSTDIR_NOT_AVAILABLE = 127;
          DAC1FID21_AIRTEMP_OFFSET = 223;
          DAC1FID21_HUMIDITY_NOT_VAILABLE = 127;
        { 920 bits of six-bit, plus NUL  }
          AIS_DAC1FID30_TEXT_MAX = 154;
          DAC1FID32_CDIR_NOT_AVAILABLE = 360;
          DAC1FID32_CSPEED_NOT_AVAILABLE = 127;
        { 952 bits  }
          AIS_TYPE8_BINARY_MAX = 952;
          DAC200FID23_TYPE_UNKNOWN = 0;
          DAC200FID23_MIN_UNKNOWN = 255;
          DAC200FID23_MAX_UNKNOWN = 255;
          DAC200FID23_CLASS_UNKNOWN = 0;
          DAC200FID23_WIND_UNKNOWN = 0;
          DAC1FID11_LATLON_SCALE = 1000;
          DAC1FID11_LON_NOT_AVAILABLE = $FFFFFF;
          DAC1FID11_LAT_NOT_AVAILABLE = $7FFFFF;
          DAC1FID11_WSPEED_NOT_AVAILABLE = 127;
          DAC1FID11_WDIR_NOT_AVAILABLE = 511;
          DAC1FID11_AIRTEMP_NOT_AVAILABLE = 2047;
          DAC1FID11_AIRTEMP_OFFSET = 600;
          DAC1FID11_AIRTEMP_DIV = 10.0;
          DAC1FID11_HUMIDITY_NOT_AVAILABLE = 127;
          DAC1FID11_DEWPOINT_NOT_AVAILABLE = 1023;
          DAC1FID11_DEWPOINT_OFFSET = 200;
          DAC1FID11_DEWPOINT_DIV = 10.0;
          DAC1FID11_PRESSURE_NOT_AVAILABLE = 511;
          DAC1FID11_PRESSURE_OFFSET = -(800);
          DAC1FID11_PRESSURETREND_NOT_AVAILABLE = 3;
          DAC1FID11_VISIBILITY_NOT_AVAILABLE = 255;
          DAC1FID11_VISIBILITY_DIV = 10.0;
          DAC1FID11_WATERLEVEL_NOT_AVAILABLE = 511;
          DAC1FID11_WATERLEVEL_OFFSET = 100;
          DAC1FID11_WATERLEVEL_DIV = 10.0;
          DAC1FID11_WATERLEVELTREND_NOT_AVAILABLE = 3;
          DAC1FID11_CSPEED_NOT_AVAILABLE = 255;
          DAC1FID11_CSPEED_DIV = 10.0;
          DAC1FID11_CDIR_NOT_AVAILABLE = 511;
          DAC1FID11_CDEPTH_NOT_AVAILABLE = 31;
          DAC1FID11_WAVEHEIGHT_NOT_AVAILABLE = 255;
          DAC1FID11_WAVEHEIGHT_DIV = 10.0;
          DAC1FID11_WAVEPERIOD_NOT_AVAILABLE = 63;
          DAC1FID11_WAVEDIR_NOT_AVAILABLE = 511;
          DAC1FID11_SEASTATE_NOT_AVAILABLE = 15;
          DAC1FID11_WATERTEMP_NOT_AVAILABLE = 1023;
          DAC1FID11_WATERTEMP_OFFSET = 100;
          DAC1FID11_WATERTEMP_DIV = 10.0;
          DAC1FID11_PRECIPTYPE_NOT_AVAILABLE = 7;
          DAC1FID11_SALINITY_NOT_AVAILABLE = 511;
          DAC1FID11_SALINITY_DIV = 10.0;
          DAC1FID11_ICE_NOT_AVAILABLE = 3;
          AIS_DAC1FID13_RADIUS_NOT_AVAILABLE = 10001;
          AIS_DAC1FID13_EXTUNIT_NOT_AVAILABLE = 0;
          DAC1FID17_IDTYPE_MMSI = 0;
          DAC1FID17_IDTYPE_IMO = 1;
          DAC1FID17_IDTYPE_CALLSIGN = 2;
          DAC1FID17_IDTYPE_OTHER = 3;
          DAC1FID17_ID_LENGTH = 7;
          DAC1FID17_COURSE_NOT_AVAILABLE = 360;
          DAC1FID17_SPEED_NOT_AVAILABLE = 255;
        { 920 bits of six-bit, plus NUL  }
          AIS_DAC1FID29_TEXT_MAX = 162;
          DAC1FID31_LATLON_SCALE = 1000;
          DAC1FID31_LON_NOT_AVAILABLE = (181*60)*DAC1FID31_LATLON_SCALE;
          DAC1FID31_LAT_NOT_AVAILABLE = (91*60)*DAC1FID31_LATLON_SCALE;
          DAC1FID31_WIND_HIGH = 126;
          DAC1FID31_WIND_NOT_AVAILABLE = 127;
          DAC1FID31_DIR_NOT_AVAILABLE = 360;
          DAC1FID31_AIRTEMP_NOT_AVAILABLE = -(1024);
          DAC1FID31_AIRTEMP_DIV = 10.0;
          DAC1FID31_HUMIDITY_NOT_AVAILABLE = 101;
          DAC1FID31_DEWPOINT_NOT_AVAILABLE = 501;
          DAC1FID31_DEWPOINT_DIV = 10.0;
          DAC1FID31_PRESSURE_NOT_AVAILABLE = 511;
          DAC1FID31_PRESSURE_HIGH = 402;
          DAC1FID31_PRESSURE_OFFSET = -(799);
          DAC1FID31_PRESSURETEND_NOT_AVAILABLE = 3;
          DAC1FID31_VISIBILITY_NOT_AVAILABLE = 127;
          DAC1FID31_VISIBILITY_DIV = 10.0;
          DAC1FID31_WATERLEVEL_NOT_AVAILABLE = 4001;
          DAC1FID31_WATERLEVEL_OFFSET = 1000;
          DAC1FID31_WATERLEVEL_DIV = 100.0;
          DAC1FID31_WATERLEVELTREND_NOT_AVAILABLE = 3;
          DAC1FID31_CSPEED_NOT_AVAILABLE = 255;
          DAC1FID31_CSPEED_DIV = 10.0;
          DAC1FID31_CDEPTH_NOT_AVAILABLE = 301;
          DAC1FID31_CDEPTH_SCALE = 10.0;
          DAC1FID31_HEIGHT_NOT_AVAILABLE = 31;
          DAC1FID31_HEIGHT_DIV = 10.0;
          DAC1FID31_PERIOD_NOT_AVAILABLE = 63;
          DAC1FID31_SEASTATE_NOT_AVAILABLE = 15;
          DAC1FID31_WATERTEMP_NOT_AVAILABLE = 601;
          DAC1FID31_WATERTEMP_DIV = 10.0;
          DAC1FID31_PRECIPTYPE_NOT_AVAILABLE = 7;
          DAC1FID31_SALINITY_NOT_AVAILABLE = 510;
          DAC1FID31_SALINITY_DIV = 10.0;
          DAC1FID31_ICE_NOT_AVAILABLE = 3;
          AIS_ALT_NOT_AVAILABLE = 4095;
        { 4094 meters or higher  }
          AIS_ALT_HIGH = 4094;
          AIS_SAR_SPEED_NOT_AVAILABLE = 1023;
          AIS_SAR_FAST_MOVER = 1022;
        { 936 bits of six-bit, plus NUL  }
          AIS_TYPE12_TEXT_MAX = 157;
        { 952 bits of six-bit, plus NUL  }
          AIS_TYPE14_TEXT_MAX = 161;
          AIS_GNSS_LATLON_DIV = 600.0;
        { 920 bits  }
          AIS_TYPE17_BINARY_MAX = 736;
          AIS_GNS_LON_NOT_AVAILABLE = $1a838;
          AIS_GNS_LAT_NOT_AVAILABLE = $d548;
          AIS_CHANNEL_LATLON_DIV = 600.0;
        { Up to 128 bits  }
          AIS_TYPE25_BINARY_MAX = 128;
        { Up to 128 bits  }
          AIS_TYPE26_BINARY_MAX = 1004;
          AIS_LONGRANGE_LATLON_DIV = 600.0;
          AIS_LONGRANGE_LON_NOT_AVAILABLE = $1a838;
          AIS_LONGRANGE_LAT_NOT_AVAILABLE = $d548;
          AIS_LONGRANGE_SPEED_NOT_AVAILABLE = 63;
          AIS_LONGRANGE_COURSE_NOT_AVAILABLE = 511;
        { message type  }
        { Repeat indicator  }
        { MMSI  }
        {    union  }
        { Types 1-3 Common navigation info  }
        { navigation status  }
        { rate of turn  }
        { speed over ground in deciknots  }
        { position accuracy  }
        { longitude  }
        { latitude  }
        { course over ground  }
        { true heading  }
        { seconds of UTC timestamp  }
        { maneuver indicator  }
        {unsigned int spare;	spare bits */ }
        { RAIM flag  }
        { radio status bits  }
        { Type 4 - Base Station Report & Type 11 - UTC and Date Response  }
        { UTC year  }
        { UTC month  }
        { UTC day  }
        { UTC hour  }
        { UTC minute  }
        { UTC second  }
        { fix quality  }
        { longitude  }
        { latitude  }
        { type of position fix device  }
        {unsigned int spare;	spare bits */ }
        { RAIM flag  }
        { radio status bits  }
        { Type 5 - Ship static and voyage related data  }
        { AIS version level  }
        { IMO identification  }
        { callsign  }
        { vessel name  }
        { ship type code  }
        { dimension to bow  }
        { dimension to stern  }
        { dimension to port  }
        { dimension to starboard  }
        { type of position fix deviuce  }
        { UTC month  }
        { UTC day  }
        { UTC hour  }
        { UTC minute  }
        { draft in meters  }
        { ship destination  }
        { data terminal enable  }
        {unsigned int spare;	spare bits */ }
        { Type 6 - Addressed Binary Message  }
        { sequence number  }
        { destination MMSI  }
        { retransmit flag  }
        {unsigned int spare;	spare bit(s) */ }
        { Application ID  }
        { Functional ID  }
        { bit count of the data  }
        {	    union  }
        { Inland AIS - ETA at lock/bridge/terminal  }
        { UN Country Code  }
        { UN/LOCODE  }
        { Fairway section  }
        { Terminal code  }
        { Fairway hectometre  }
        { ETA month  }
        { ETA day  }
        { ETA hour  }
        { ETA minute  }
        { Assisting Tugs  }
        { Air Draught  }
        { Inland AIS - ETA at lock/bridge/terminal  }
        { UN Country Code  }
        { UN/LOCODE  }
        { Fairway section  }
        { Terminal code  }
        { Fairway hectometre  }
        { RTA month  }
        { RTA day  }
        { RTA hour  }
        { RTA minute  }
        { Status  }
        { Inland AIS - Number of persons on board  }
        { # crew on board  }
        { # passengers on board  }
        { # personnel on board  }
        { GLA - AtoN monitoring data (UK/ROI)  }
        { Analogue (internal)  }
        { Analogue (external #1)  }
        { Analogue (external #2)  }
        { RACON status  }
        { Light status  }
        { Health alarm }
        { Status bits (external)  }
        { Off position status  }
        { IMO236 - Dangerous Cargo Indication  }
        { Last Port Of Call  }
        { ETA month  }
        { ETA day  }
        { ETA hour  }
        { ETA minute  }
        { Next Port Of Call  }
        { ETA month  }
        { ETA day  }
        { ETA hour  }
        { ETA minute  }
        { Main Dangerous Good  }
        { IMD Category  }
        { UN Number  }
        { Amount of Cargo  }
        { Unit of Quantity  }
        { IMO236 - Extended Ship Static and Voyage Related Data  }
        { Air Draught  }
        { IMO236 - Number of Persons on board  }
        { number of persons  }
        { IMO289 - Clearance Time To Enter Port  }
        { Message Linkage ID  }
        { Month (UTC)  }
        { Day (UTC)  }
        { Hour (UTC)  }
        { Minute (UTC)  }
        { Name of Port & Berth  }
        { Destination  }
        { Longitude  }
        { Latitude  }
        { IMO289 - Berthing Data (addressed)  }
        { Message Linkage ID  }
        { Berth length  }
        { Berth Water Depth  }
        { Mooring Position  }
        { Month (UTC)  }
        { Day (UTC)  }
        { Hour (UTC)  }
        { Minute (UTC)  }
        { Services Availability  }
        { Agent  }
        { Bunker/fuel  }
        { Chandler  }
        { Stevedore  }
        { Electrical  }
        { Potable water  }
        { Customs house  }
        { Cartage  }
        { Crane(s)  }
        { Lift(s)  }
        { Medical facilities  }
        { Navigation repair  }
        { Provisions  }
        { Ship repair  }
        { Surveyor  }
        { Steam  }
        { Tugs  }
        { Waste disposal (solid)  }
        { Waste disposal (liquid)  }
        { Waste disposal (hazardous)  }
        { Reserved ballast exchange  }
        { Additional services  }
        { Regional reserved 1  }
        { Regional reserved 2  }
        { Reserved for future  }
        { Reserved for future  }
        { Name of Berth  }
        { Longitude  }
        { Latitude  }
        { IMO289 - Weather observation report from ship  }
        {** WORK IN PROGRESS - NOT YET DECODED ** }
        { true if WMO variant  }
        {		    union  }
        { Location  }
        { Longitude  }
        { Latitude  }
        { Report day  }
        { Report hour  }
        { Report minute  }
        { Max range?  }
        { Units of 0.1 nm  }
        { units of 1%  }
        { average wind speed  }
        { wind gust  }
        { wind direction  }
        { air pressure, hpa  }
        { tendency  }
        { temp, units 0.1C  }
        { units 0.1degC  }
        { in seconds  }
        { direction in deg  }
        { in decimeters  }
        { in seconds  }
        { direction in deg  }
        { Longitude  }
        { Latitude  }
        { UTC month  }
        { Report day  }
        { Report hour  }
        { Report minute  }
        { course over ground  }
        { speed, m/s  }
        { true heading  }
        { units of hPa * 0.1  }
        { units of hPa * 0.1  }
        { enumerated  }
        { in 5 degree steps  }
        { meters per second  }
        { in 5 degree steps  }
        { meters per second  }
        { meters per second  }
        { in 5 degree steps  }
        { degress K  }
        { units of 1%  }
        { some trailing fields are missing  }
        {		    ; }
        {** WORK IN PROGRESS ENDS HERE ** }
        { IMO289 - Dangerous Cargo Indication  }
        { Unit of Quantity  }
        { Amount of Cargo  }
        { Cargo code  }
        { Cargo subtype  }
        { IMO289 - Route info (addressed)  }
        { IMO289 - Text message (addressed)  }
        { IMO289 & IMO236 - Tidal Window  }
        { Month  }
        { Day  }
        { Longitude  }
        { Latitude  }
        { From UTC Hour  }
        { From UTC Minute  }
        { To UTC Hour  }
        { To UTC Minute  }
        { Current Dir. Predicted  }
        { Current Speed Predicted  }
        {	    ; }
        { Type 7 - Binary Acknowledge  }
        { spares ignored, they're only padding here  }
        { Type 8 - Broadcast Binary Message  }
        { Designated Area Code  }
        { Functional ID  }
        { bit count of the data  }
        {	    union  }
        { Inland static ship and voyage-related data  }
        { European Vessel ID  }
        { Length of ship  }
        { Beam of ship  }
        { Ship/combination type  }
        { Hazardous cargo  }
        { Draught  }
        { Loaded/Unloaded  }
        { Speed inf. quality  }
        { Course inf. quality  }
        { Heading inf. quality  }
        { Inland AIS EMMA Warning  }
        { Start Year  }
        { Start Month  }
        { Start Day  }
        { End Year  }
        { End Month  }
        { End Day  }
        { Start Hour  }
        { Start Minute  }
        { End Hour  }
        { End Minute  }
        { Start Longitude  }
        { Start Latitude  }
        { End Longitude  }
        { End Latitude  }
        { Type  }
        { Min value  }
        { Max value  }
        { Classification  }
        { Wind Direction  }
        { UN Country Code  }
        { Gauge ID  }
        { Water Level  }
        { Signal Longitude  }
        { Signal Latitude  }
        { Signal form  }
        { Signal orientation  }
        { Direction of impact  }
        { Light Status  }
        { IMO236  - Meteorological-Hydrological data
        		 * Trial message, not to be used after January 2013
        		 * Replaced by IMO289 (DAC 1, FID 31)
        		  }
        { longitude in minutes * .001  }
        { latitude in minutes * .001  }
        { UTC day  }
        { UTC hour  }
        { UTC minute  }
        { average wind speed  }
        { wind gust  }
        { wind direction  }
        { wind gust direction  }
        { temperature, units 0.1C  }
        { relative humidity, %  }
        { dew point, units 0.1C  }
        { air pressure, hpa  }
        { tendency  }
        { units 0.1 nautical miles  }
        { decimeters  }
        { water level trend code  }
        { surface current speed in deciknots  }
        { surface current dir., degrees  }
        { current speed in deciknots  }
        { current dir., degrees  }
        { measurement depth, m  }
        { current speed in deciknots  }
        { current dir., degrees  }
        { measurement depth, m  }
        { in decimeters  }
        { in seconds  }
        { direction in degrees  }
        { in decimeters  }
        { in seconds  }
        { direction in degrees  }
        { Beaufort scale, 0-12  }
        { units 0.1deg Celsius  }
        { 0-7, enumerated  }
        { units of 0.1ppt  }
        { is there sea ice?  }
        { IMO236 - Fairway Closed  }
        { Reason For Closing  }
        { Location Of Closing From  }
        { Location of Closing To  }
        { Radius extension  }
        { Unit of extension  }
        { From day (UTC)  }
        { From month (UTC)  }
        { From hour (UTC)  }
        { From minute (UTC)  }
        { To day (UTC)  }
        { To month (UTC)  }
        { To hour (UTC)  }
        { To minute (UTC)  }
        { IMO236 - Extended ship and voyage data  }
        { Air Draught  }
        { IMO286 - Number of Persons on board  }
        { number of persons  }
        { IMO289 - VTS-generated/Synthetic Targets  }
        { Identifier type  }
        { Target identifier  }
        { Latitude  }
        { Longitude  }
        { Course Over Ground  }
        { Time Stamp  }
        { Speed Over Ground  }
        { IMO 289 - Marine Traffic Signal  }
        { Message Linkage ID  }
        { Name of Signal Station  }
        { Longitude  }
        { Latitude  }
        { Status of Signal  }
        { Signal In Service  }
        { UTC hour  }
        { UTC minute  }
        { Expected Next Signal  }
        { IMO289 - Route info (broadcast)  }
        { IMO289 - Text message (broadcast)  }
        { IMO289 - Meteorological-Hydrological data  }
        { position accuracy, <10m if true  }
        { longitude in minutes * .001  }
        { longitude in minutes * .001  }
        { UTC day  }
        { UTC hour  }
        { UTC minute  }
        { average wind speed  }
        { wind gust  }
        { wind direction  }
        { wind gust direction  }
        { temperature, units 0.1C  }
        { relative humidity, %  }
        { dew point, units 0.1C  }
        { air pressure, hpa  }
        { tendency  }
        { visibility greater than  }
        { units 0.1 nautical miles  }
        { cm  }
        { water level trend code  }
        { current speed in deciknots  }
        { current dir., degrees  }
        { current speed in deciknots  }
        { current dir., degrees  }
        { measurement depth, 0.1m  }
        { current speed in deciknots  }
        { current dir., degrees  }
        { measurement depth, 0.1m  }
        { in decimeters  }
        { in seconds  }
        { direction in degrees  }
        { in decimeters  }
        { in seconds  }
        { direction in degrees  }
        { Beaufort scale, 0-12  }
        { units 0.1deg Celsius  }
        { 0-7, enumerated  }
        { units of 0.1 permil (ca. PSU)  }
        { is there sea ice?  }
        {	    ; }
        { Type 9 - Standard SAR Aircraft Position Report  }
        { altitude in meters  }
        { speed over ground in deciknots  }
        { position accuracy  }
        { longitude  }
        { latitude  }
        { course over ground  }
        { seconds of UTC timestamp  }
        { regional reserved  }
        { data terminal enable  }
        {unsigned int spare;	spare bits */ }
        { assigned-mode flag  }
        { RAIM flag  }
        { radio status bits  }
        { Type 10 - UTC/Date Inquiry  }
        {unsigned int spare; }
        { destination MMSI  }
        {unsigned int spare2; }
        { Type 12 - Safety-Related Message  }
        { sequence number  }
        { destination MMSI  }
        { retransmit flag  }
        {unsigned int spare;	spare bit(s) */ }
        { Type 14 - Safety-Related Broadcast Message  }
        {unsigned int spare;	spare bit(s) */ }
        { Type 15 - Interrogation  }
        {unsigned int spare;	spare bit(s) */ }
        {unsigned int spare2;	spare bit(s) */ }
        {unsigned int spare3;	spare bit(s) */ }
        {unsigned int spare4;	spare bit(s) */ }
        { Type 16 - Assigned Mode Command  }
        {unsigned int spare;	spare bit(s) */ }
        { Type 17 - GNSS Broadcast Binary Message  }
        {unsigned int spare;	spare bit(s) */ }
        { longitude  }
        { latitude  }
        {unsigned int spare2;	spare bit(s) */ }
        { bit count of the data  }
        { Type 18 - Standard Class B CS Position Report  }
        { altitude in meters  }
        { speed over ground in deciknots  }
        { position accuracy  }
        { longitude  }
        { latitude  }
        { course over ground  }
        { true heading  }
        { seconds of UTC timestamp  }
        { regional reserved  }
        { carrier sense unit flag  }
        { unit has attached display?  }
        { unit attached to radio with DSC?  }
        { unit can switch frequency bands?  }
        { can accept Message 22 management?  }
        { assigned-mode flag  }
        { RAIM flag  }
        { radio status bits  }
        { Type 19 - Extended Class B CS Position Report  }
        { altitude in meters  }
        { speed over ground in deciknots  }
        { position accuracy  }
        { longitude  }
        { latitude  }
        { course over ground  }
        { true heading  }
        { seconds of UTC timestamp  }
        { regional reserved  }
        { ship name  }
        { ship type code  }
        { dimension to bow  }
        { dimension to stern  }
        { dimension to port  }
        { dimension to starboard  }
        { type of position fix deviuce  }
        { RAIM flag  }
        { date terminal enable  }
        { assigned-mode flag  }
        {unsigned int spare;	spare bits */ }
        { Type 20 - Data Link Management Message  }
        {unsigned int spare;	spare bit(s) */ }
        { TDMA slot offset  }
        { number of xlots to allocate  }
        { allocation timeout  }
        { repeat increment  }
        { TDMA slot offset  }
        { number of xlots to allocate  }
        { allocation timeout  }
        { repeat increment  }
        { TDMA slot offset  }
        { number of xlots to allocate  }
        { allocation timeout  }
        { repeat increment  }
        { TDMA slot offset  }
        { number of xlots to allocate  }
        { allocation timeout  }
        { repeat increment  }
        { Type 21 - Aids to Navigation Report  }
        { aid type  }
        { name of aid to navigation  }
        { position accuracy  }
        { longitude  }
        { latitude  }
        { dimension to bow  }
        { dimension to stern  }
        { dimension to port  }
        { dimension to starboard  }
        { type of EPFD  }
        { second of UTC timestamp  }
        { off-position indicator  }
        { regional reserved field  }
        { RAIM flag  }
        { is virtual station?  }
        { assigned-mode flag  }
        {unsigned int spare;	unused */ }
        { Type 22 - Channel Management  }
        {unsigned int spare;	spare bit(s) */ }
        { Channel A number  }
        { Channel B number  }
        { transmit/receive mode  }
        { high-power flag  }
        {	    union  }
        { NE corner longitude  }
        { NE corner latitude  }
        { SW corner longitude  }
        { SW corner latitude  }
        { addressed station MMSI 1  }
        { addressed station MMSI 2  }
        {	    ; }
        { addressed vs. broadast flag  }
        { fix 1.5kHz band for channel A  }
        { fix 1.5kHz band for channel B  }
        { size of transitional zone  }
        { Type 23 - Group Assignment Command  }
        { NE corner longitude  }
        { NE corner latitude  }
        { SW corner longitude  }
        { SW corner latitude  }
        {unsigned int spare;	spare bit(s) */ }
        { station type code  }
        { ship type code  }
        {unsigned int spare2;	spare bit(s) */ }
        { transmit-enable code  }
        { report interval  }
        { quiet time  }
        {unsigned int spare3;	spare bit(s) */ }
        { Type 24 - Class B CS Static Data Report  }
        { vessel name  }
        { ship type code  }
        { vendor ID  }
        { unit model code  }
        { serial number  }
        { callsign  }
        {	    union  }
        { MMSI of main vessel  }
        { dimension to bow  }
        { dimension to stern  }
        { dimension to port  }
        { dimension to starboard  }
        {	    ; }
        { Type 25 - Addressed Binary Message  }
        { addressed-vs.broadcast flag  }
        { structured-binary flag  }
        { destination MMSI  }
        { Application ID  }
        { bit count of the data  }
        { Type 26 - Addressed Binary Message  }
        { addressed-vs.broadcast flag  }
        { structured-binary flag  }
        { destination MMSI  }
        { Application ID  }
        { bit count of the data  }
        { radio status bits  }
        { Type 27 - Long Range AIS Broadcast message  }
        { position accuracy  }
        { RAIM flag  }
        { navigation status  }
        { longitude  }
        { latitude  }
        { speed over ground in deciknots  }
        { course over ground  }
        { are we reporting GNSS position?  }
        {    ; }

        type
          ais_t = record
              _type : dword;
              _repeat : dword;
              mmsi : dword;
          case longint of
            1 : (
              type1 : record
                  status : dword;
                  turn : longint;
                  speed : dword;
                  accuracy : boolean;
                  lon : longint;
                  lat : longint;
                  course : dword;
                  heading : dword;
                  second : dword;
                  maneuver : dword;
                  raim : boolean;
                  radio : dword;
                end;
                );
            2 : (
              type4 : record
                  year : dword;
                  month : dword;
                  day : dword;
                  hour : dword;
                  minute : dword;
                  second : dword;
                  accuracy : boolean;
                  lon : longint;
                  lat : longint;
                  epfd : dword;
                  raim : boolean;
                  radio : dword;
                end;
                );
            3 : (
              type5 : record
                  ais_version : dword;
                  imo : dword;
                  callsign : array[0..(7+1)-1] of char;
                  shipname : array[0..(AIS_SHIPNAME_MAXLEN+1)-1] of char;
                  shiptype : dword;
                  to_bow : dword;
                  to_stern : dword;
                  to_port : dword;
                  to_starboard : dword;
                  epfd : dword;
                  month : dword;
                  day : dword;
                  hour : dword;
                  minute : dword;
                  draught : dword;
                  destination : array[0..(20+1)-1] of char;
                  dte : dword;
                end;
                );
            4 : (
              type6 : record
                  seqno : dword;
                  dest_mmsi : dword;
                  retransmit : boolean;
                  dac : dword;
                  fid : dword;
                  bitcount : size_t;
              case longint of
                0 : (
                  bitdata : array[0..((AIS_TYPE6_BINARY_MAX+7) div 8)-1] of char;
                  );
                1 : (
                  dac200fid21 : record
                      country : array[0..(2+1)-1] of char;
                      locode : array[0..(3+1)-1] of char;
                      section : array[0..(5+1)-1] of char;
                      terminal : array[0..(5+1)-1] of char;
                      hectometre : array[0..(5+1)-1] of char;
                      month : dword;
                      day : dword;
                      hour : dword;
                      minute : dword;
                      tugs : dword;
                      airdraught : dword;
                    end;
                    );
                2 : (
                  dac200fid22 : record
                      country : array[0..(2+1)-1] of char;
                      locode : array[0..(3+1)-1] of char;
                      section : array[0..(5+1)-1] of char;
                      terminal : array[0..(5+1)-1] of char;
                      hectometre : array[0..(5+1)-1] of char;
                      month : dword;
                      day : dword;
                      hour : dword;
                      minute : dword;
                      status : dword;
                    end;
                    );
                3 : (
                  dac200fid55 : record
                      crew : dword;
                      passengers : dword;
                      personnel : dword;
                    end;
                    );
                4 : (
                  dac235fid10 : record
                      ana_int : dword;
                      ana_ext1 : dword;
                      ana_ext2 : dword;
                      racon : dword;
                      light : dword;
                      alarm : boolean;
                      stat_ext : dword;
                      off_pos : boolean;
                    end;
                    );
                5 : (
                  dac1fid12 : record
                      lastport : array[0..(5+1)-1] of char;
                      lmonth : dword;
                      lday : dword;
                      lhour : dword;
                      lminute : dword;
                      nextport : array[0..(5+1)-1] of char;
                      nmonth : dword;
                      nday : dword;
                      nhour : dword;
                      nminute : dword;
                      dangerous : array[0..(20+1)-1] of char;
                      imdcat : array[0..(4+1)-1] of char;
                      unid : dword;
                      amount : dword;
                      _unit : dword;
                    end;
                    );
                6 : (
                  dac1fid15 : record
                      airdraught : dword;
                    end;
                    );
                7 : (
                  dac1fid16 : record
                      persons : dword;
                    end;
                    );
                8 : (
                  dac1fid18 : record
                      linkage : dword;
                      month : dword;
                      day : dword;
                      hour : dword;
                      minute : dword;
                      portname : array[0..(20+1)-1] of char;
                      destination : array[0..(5+1)-1] of char;
                      lon : longint;
                      lat : longint;
                    end;
                    );
                9 : (
                  dac1fid20 : record
                      linkage : dword;
                      berth_length : dword;
                      berth_depth : dword;
                      position : dword;
                      month : dword;
                      day : dword;
                      hour : dword;
                      minute : dword;
                      availability : dword;
                      agent : dword;
                      fuel : dword;
                      chandler : dword;
                      stevedore : dword;
                      electrical : dword;
                      water : dword;
                      customs : dword;
                      cartage : dword;
                      crane : dword;
                      lift : dword;
                      medical : dword;
                      navrepair : dword;
                      provisions : dword;
                      shiprepair : dword;
                      surveyor : dword;
                      steam : dword;
                      tugs : dword;
                      solidwaste : dword;
                      liquidwaste : dword;
                      hazardouswaste : dword;
                      ballast : dword;
                      additional : dword;
                      regional1 : dword;
                      regional2 : dword;
                      future1 : dword;
                      future2 : dword;
                      berth_name : array[0..(20+1)-1] of char;
                      berth_lon : longint;
                      berth_lat : longint;
                    end;
                    );
                10 : (
                  dac1fid21 : record
                      wmo : boolean;
                   case longint of
                    1 : (
                      nonwmo_obs : record
                          location : array[0..(20+1)-1] of char;
                          lon : longint;
                          lat : longint;
                          day : dword;
                          hour : dword;
                          minute : dword;
                          vislimit : boolean;
                          visibility : dword;
                          humidity : dword;
                          wspeed : dword;
                          wgust : dword;
                          wdir : dword;
                          pressure : dword;
                          pressuretend : dword;
                          airtemp : longint;
                          watertemp : dword;
                          waveperiod : dword;
                          wavedir : dword;
                          swellheight : dword;
                          swellperiod : dword;
                          swelldir : dword;
                        end;
                        );
                      2 : (
                        wmo_obs : record
                          lon : longint;
                          lat : longint;
                          month : dword;
                          day : dword;
                          hour : dword;
                          minute : dword;
                          course : dword;
                          speed : dword;
                          heading : dword;
                          pressure : dword;
                          pdelta : dword;
                          ptend : dword;
                          twinddir : dword;
                          twindspeed : dword;
                          rwinddir : dword;
                          rwindspeed : dword;
                          mgustspeed : dword;
                          mgustdir : dword;
                          airtemp : dword;
                          humidity : dword;
                        end;
                        );
                    end;
                    );
                11 : (
                  dac1fid25 : record
                      _unit : dword;
                      amount : dword;
                      ncargos : longint;
                      cargos : array[0..27] of record
                          code : dword;
                          subtype : dword;
                        end;
                    end;
                    );
                12 : ( dac1fid28 : route_info; );
                13 : (
                  dac1fid30 : record
                      linkage : dword;
                      text : array[0..(AIS_DAC1FID30_TEXT_MAX)-1] of char;
                    end;
                    );
                14 : (
                  dac1fid32 : record
                      month : dword;
                      day : dword;
                      ntidals : longint;
                      tidals : array[0..2] of record
                          lon : longint;
                          lat : longint;
                          from_hour : dword;
                          from_min : dword;
                          to_hour : dword;
                          to_min : dword;
                          cdir : dword;
                          cspeed : dword;
                        end;
                    end;
                    );
                end;
                );
            5 : (
              type7 : record
                  mmsi1 : dword;
                  mmsi2 : dword;
                  mmsi3 : dword;
                  mmsi4 : dword;
                end;
                );
            6 : (
              type8 : record
                  dac : dword;
                  fid : dword;
                  bitcount : size_t;
               case longint of
                0 : (
                  bitdata : array[0..((AIS_TYPE8_BINARY_MAX+7) div 8)-1] of char;
                  );
                1 : (
                  dac200fid10 : record
                      vin : array[0..(8+1)-1] of char;
                      length : dword;
                      beam : dword;
                      shiptype : dword;
                      hazard : dword;
                      draught : dword;
                      loaded : dword;
                      speed_q : boolean;
                      course_q : boolean;
                      heading_q : boolean;
                    end;
                    );
                2 : (
                  dac200fid23 : record
                      start_year : dword;
                      start_month : dword;
                      start_day : dword;
                      end_year : dword;
                      end_month : dword;
                      end_day : dword;
                      start_hour : dword;
                      start_minute : dword;
                      end_hour : dword;
                      end_minute : dword;
                      start_lon : longint;
                      start_lat : longint;
                      end_lon : longint;
                      end_lat : longint;
                      _type : dword;
                      min : longint;
                      max : longint;
                      intensity : dword;
                      wind : dword;
                    end;
                    );
                3 : (
                  dac200fid24 : record
                      country : array[0..(2+1)-1] of char;
                      ngauges : longint;
                      gauges : array[0..3] of record
                          id : dword;
                          level : longint;
                        end;
                    end;
                    );
                4 : (
                  dac200fid40 : record
                      lon : longint;
                      lat : longint;
                      form : dword;
                      facing : dword;
                      direction : dword;
                      status : dword;
                    end;
                    );
                5 : (
                  dac1fid11 : record
                      lon : longint;
                      lat : longint;
                      day : dword;
                      hour : dword;
                      minute : dword;
                      wspeed : dword;
                      wgust : dword;
                      wdir : dword;
                      wgustdir : dword;
                      airtemp : dword;
                      humidity : dword;
                      dewpoint : dword;
                      pressure : dword;
                      pressuretend : dword;
                      visibility : dword;
                      waterlevel : longint;
                      leveltrend : dword;
                      cspeed : dword;
                      cdir : dword;
                      cspeed2 : dword;
                      cdir2 : dword;
                      cdepth2 : dword;
                      cspeed3 : dword;
                      cdir3 : dword;
                      cdepth3 : dword;
                      waveheight : dword;
                      waveperiod : dword;
                      wavedir : dword;
                      swellheight : dword;
                      swellperiod : dword;
                      swelldir : dword;
                      seastate : dword;
                      watertemp : dword;
                      preciptype : dword;
                      salinity : dword;
                      ice : dword;
                    end;
                    );
                6 : (
                  dac1fid13 : record
                      reason : array[0..(20+1)-1] of char;
                      closefrom : array[0..(20+1)-1] of char;
                      closeto : array[0..(20+1)-1] of char;
                      radius : dword;
                      extunit : dword;
                      fday : dword;
                      fmonth : dword;
                      fhour : dword;
                      fminute : dword;
                      tday : dword;
                      tmonth : dword;
                      thour : dword;
                      tminute : dword;
                    end;
                    );
                7 : (
                  dac1fid15 : record
                      airdraught : dword;
                    end;
                    );
                8 : (
                  dac1fid16 : record
                      persons : dword;
                    end;
                    );
                9 : (
                  dac1fid17 : record
                      ntargets : longint;
                      targets : array[0..3] of record
                          idtype : dword;
                          id : record
                              case longint of
                                0 : ( mmsi : dword );
                                1 : ( imo : dword );
                                2 : ( callsign : array[0..(DAC1FID17_ID_LENGTH+1)-1] of char );
                                3 : ( other : array[0..(DAC1FID17_ID_LENGTH+1)-1] of char );
                              end;
                          lat : longint;
                          lon : longint;
                          course : dword;
                          second : dword;
                          speed : dword;
                        end;
                    end;
                    );
                10 : (
                  dac1fid19 : record
                      linkage : dword;
                      station : array[0..(20+1)-1] of char;
                      lon : longint;
                      lat : longint;
                      status : dword;
                      signal : dword;
                      hour : dword;
                      minute : dword;
                      nextsignal : dword;
                    end;
                    );
                11 : (
                  dac1fid27 : route_info;
                  );
                12 : (
                  dac1fid29 : record
                      linkage : dword;
                      text : array[0..(AIS_DAC1FID29_TEXT_MAX)-1] of char;
                    end;
                    );
                13 : (
                  dac1fid31 : record
                      accuracy : boolean;
                      lon : longint;
                      lat : longint;
                      day : dword;
                      hour : dword;
                      minute : dword;
                      wspeed : dword;
                      wgust : dword;
                      wdir : dword;
                      wgustdir : dword;
                      airtemp : longint;
                      humidity : dword;
                      dewpoint : longint;
                      pressure : dword;
                      pressuretend : dword;
                      visgreater : boolean;
                      visibility : dword;
                      waterlevel : longint;
                      leveltrend : dword;
                      cspeed : dword;
                      cdir : dword;
                      cspeed2 : dword;
                      cdir2 : dword;
                      cdepth2 : dword;
                      cspeed3 : dword;
                      cdir3 : dword;
                      cdepth3 : dword;
                      waveheight : dword;
                      waveperiod : dword;
                      wavedir : dword;
                      swellheight : dword;
                      swellperiod : dword;
                      swelldir : dword;
                      seastate : dword;
                      watertemp : longint;
                      preciptype : dword;
                      salinity : dword;
                      ice : dword;
                    end;
                    );
                end;
                );
            7 : (
              type9 : record
                  alt : dword;
                  speed : dword;
                  accuracy : boolean;
                  lon : longint;
                  lat : longint;
                  course : dword;
                  second : dword;
                  regional : dword;
                  dte : dword;
                  assigned : boolean;
                  raim : boolean;
                  radio : dword;
                end;
                );
            8 : (
              type10 : record
                  dest_mmsi : dword;
                end;
                );
            9 : (
              type12 : record
                  seqno : dword;
                  dest_mmsi : dword;
                  retransmit : boolean;
                  text : array[0..(AIS_TYPE12_TEXT_MAX)-1] of char;
                end;
                );
            10 : (
              type14 : record
                  text : array[0..(AIS_TYPE14_TEXT_MAX)-1] of char;
                end;
                );
            11 : (
              type15 : record
                  mmsi1 : dword;
                  type1_1 : dword;
                  offset1_1 : dword;
                  type1_2 : dword;
                  offset1_2 : dword;
                  mmsi2 : dword;
                  type2_1 : dword;
                  offset2_1 : dword;
                end;
                );
            12 : (
              type16 : record
                  mmsi1 : dword;
                  offset1 : dword;
                  increment1 : dword;
                  mmsi2 : dword;
                  offset2 : dword;
                  increment2 : dword;
                end;
                );
            13 : (
              type17 : record
                  lon : longint;
                  lat : longint;
                  bitcount : size_t;
                  bitdata : array[0..((AIS_TYPE17_BINARY_MAX+7) div 8)-1] of char;
                end;
                );
            14 : (
              type18 : record
                  reserved : dword;
                  speed : dword;
                  accuracy : boolean;
                  lon : longint;
                  lat : longint;
                  course : dword;
                  heading : dword;
                  second : dword;
                  regional : dword;
                  cs : boolean;
                  display : boolean;
                  dsc : boolean;
                  band : boolean;
                  msg22 : boolean;
                  assigned : boolean;
                  raim : boolean;
                  radio : dword;
                end;
                );
            15 : (
              type19 : record
                  reserved : dword;
                  speed : dword;
                  accuracy : boolean;
                  lon : longint;
                  lat : longint;
                  course : dword;
                  heading : dword;
                  second : dword;
                  regional : dword;
                  shipname : array[0..(AIS_SHIPNAME_MAXLEN+1)-1] of char;
                  shiptype : dword;
                  to_bow : dword;
                  to_stern : dword;
                  to_port : dword;
                  to_starboard : dword;
                  epfd : dword;
                  raim : boolean;
                  dte : dword;
                  assigned : boolean;
                end;
                );
            16 : (
              type20 : record
                  offset1 : dword;
                  number1 : dword;
                  timeout1 : dword;
                  increment1 : dword;
                  offset2 : dword;
                  number2 : dword;
                  timeout2 : dword;
                  increment2 : dword;
                  offset3 : dword;
                  number3 : dword;
                  timeout3 : dword;
                  increment3 : dword;
                  offset4 : dword;
                  number4 : dword;
                  timeout4 : dword;
                  increment4 : dword;
                end;
                );
            17 : (
              type21 : record
                  aid_type : dword;
                  name : array[0..34] of char;
                  accuracy : boolean;
                  lon : longint;
                  lat : longint;
                  to_bow : dword;
                  to_stern : dword;
                  to_port : dword;
                  to_starboard : dword;
                  epfd : dword;
                  second : dword;
                  off_position : boolean;
                  regional : dword;
                  raim : boolean;
                  virtual_aid : boolean;
                  assigned : boolean;
                end;
                );
            18 : (
              type22 : record
                  channel_a : dword;
                  channel_b : dword;
                  txrx : dword;
                  power : boolean;
               case longint of
                1 : (
                  area : record
                      ne_lon : longint;
                      ne_lat : longint;
                      sw_lon : longint;
                      sw_lat : longint;
                    end;
                    addressed : boolean;
                    band_a : boolean;
                    band_b : boolean;
                    zonesize : dword;
                    );
                2 : (
                  mmsi : record
                      dest1 : dword;
                      dest2 : dword;
                              addressed : boolean;
                              band_a : boolean;
                              band_b : boolean;
                              zonesize : dword;
                    end;
                    );
                end;
                );
            19 : (
              type23 : record
                  ne_lon : longint;
                  ne_lat : longint;
                  sw_lon : longint;
                  sw_lat : longint;
                  stationtype : dword;
                  shiptype : dword;
                  txrx : dword;
                  interval : dword;
                  quiet : dword;
                end;
                );
            20 : (
              type24 : record
                  shipname : array[0..(AIS_SHIPNAME_MAXLEN+1)-1] of char;
                  part : (both,part_a,part_b);
                  shiptype : dword;
                  vendorid : array[0..7] of char;
                  model : dword;
                  serial : dword;
                  callsign : array[0..7] of char;
                  mothership_mmsi : dword;
                  dim : record
                      to_bow : dword;
                      to_stern : dword;
                      to_port : dword;
                      to_starboard : dword;
                    end;
                end;
                );
            21 : (
              type25 : record
                  addressed : boolean;
                  structured : boolean;
                  dest_mmsi : dword;
                  app_id : dword;
                  bitcount : size_t;
                  bitdata : array[0..((AIS_TYPE25_BINARY_MAX+7) div 8)-1] of char;
                end;
                );
            22 : (
              type26 : record
                  addressed : boolean;
                  structured : boolean;
                  dest_mmsi : dword;
                  app_id : dword;
                  bitcount : size_t;
                  bitdata : array[0..((AIS_TYPE26_BINARY_MAX+7) div 8)-1] of char;
                  radio : dword;
                end;
                );
            23 : (
              type27 : record
                  accuracy : boolean;
                  raim : boolean;
                  status : dword;
                  lon : longint;
                  lat : longint;
                  speed : dword;
                  course : dword;
                  gnss : boolean;
                end;
                );
            end;


    { has field been set since this was last cleared?  }
    { NZ if GPS is on line, 0 if not.
    				 *
    				 * Note: gpsd clears this time when sentences
    				 * fail to show up within the GPS's normal
    				 * send cycle time. If the host-to-GPS
    				 * link is lossy enough to drop entire
    				 * sentences, this field will be
    				 * prone to false zero values.
    				  }
    { socket or file descriptor to GPS  }
    { accumulated PVT data  }
    { this should move to the per-driver structure  }
    { Geoidal separation, MSL - WGS84 (Meters)  }
    { GPS status -- always valid  }
    { Do we have a fix?  }
    { precision of fix -- valid if satellites_used > 0  }
    { Number of satellites used in solution  }
    { PRNs of satellites used in solution  }
    { redundant with the estimate elements in the fix structure  }
    { spherical position error, 95% confidence (meters)   }
    { satellite status -- valid when satellites_visible > 0  }
    { skyview timestamp  }
    { # of satellites in view  }
    { PRNs of satellite  }
    { elevation of satellite  }
    { azimuth  }
    { signal-to-noise ratio (dB)  }
    { device that shipped last update  }
    { our listening policy  }
    { should be moved to privdata someday  }
    { tag of last sentence processed  }
    { pack things never reported together to reduce structure size  }
    {    union  }
    { unusual forms of sensor data that might come up the pipe  }
    { "artificial" structures for various protocol responses  }
    {    ; }
    { Private data - client code must not set this  }

    type
      gps_data_t = record
          maskset : gps_mask_t;
          online : timestamp_t;
          gps_fd : socket_t;
          fix : gps_fix_t;
          separation : double;
          status : longint;
          satellites_used : longint;
          used : array[0..(MAXCHANNELS)-1] of longint;
          dop : dop_t;
          epe : double;
          skyview_time : timestamp_t;
          satellites_visible : longint;
          PRN : array[0..(MAXCHANNELS)-1] of longint;
          elevation : array[0..(MAXCHANNELS)-1] of longint;
          azimuth : array[0..(MAXCHANNELS)-1] of longint;
          ss : array[0..(MAXCHANNELS)-1] of double;
          dev : devconfig_t;
          policy : policy_t;
          tag : array[0..(MAXTAGLEN+1)-1] of char;
       case longint of
        1 : (
          rtcm2 : rtcm2_t;
          );
        2 : (
          rtcm3 : rtcm3_t;
          );
        3 : (
          subframe : subframe_t;
          );
        4 : (
          ais : ais_t;
          );
        5 : (
          attitude : attitude_t;
          );
        6 : (
          raw : rawdata_t;
          );
        7 : (
          gst : gst_t;
          );
        8 : (
          version : version_t;
          );
        9 : (
          devices : record
              time : timestamp_t;
              ndevices : longint;
              list : array[0..(MAXUSERDEVS)-1] of devconfig_t;
            end;
          );
        10 : (
          error : array[0..255] of char;
          );
        11 : (
          timedrift : timedrift_t;
          );
        end;
  Pchar  = ^char;
  Pdop_t  = ^dop_t;
  Pdouble  = ^double;
  PFILE  = ^FILE;
  Pgps_data_t  = ^gps_data_t;
  Pgps_fix_t  = ^gps_fix_t;
{$IFDEF FPC}
{$PACKRECORDS C}
{$ENDIF}


  {
   * The structure describing an uncertainty volume in kinematic space.
   * This is what GPSes are meant to produce; all the other info is
   * technical impedimenta.
   *
   * All double values use NAN to indicate data not available.
   *
   * Usually all the information in this structure was considered valid
   * by the GPS at the time of update.  This will be so if you are using
   * a GPS chipset that speaks SiRF binary, Garmin binary, or Zodiac binary.
   * This covers over 80% of GPS products in early 2005.
   *
   * If you are using a chipset that speaks NMEA, this structure is updated
   * in bits by GPRMC (lat/lon, track, speed), GPGGA (alt, climb), GPGLL
   * (lat/lon), and GPGSA (eph, epv).  Most NMEA GPSes take a single fix
   * at the beginning of a 1-second cycle and report the same timestamp in
   * GPRMC, GPGGA, and GPGLL; for these, all info is guaranteed correctly
   * synced to the time member, but you'll get different stages of the same
   * update depending on where in the cycle you poll.  A very few GPSes,
   * like the Garmin 48, take a new fix before more than one of of
   * GPRMC/GPGGA/GPGLL during a single cycle; thus, they may have different
   * timestamps and some data in this structure can be up to 1 cycle (usually
   * 1 second) older than the fix time.
   *
   * Error estimates are at 95% confidence.
    }



  {
   * Satellite ID classes.
   * IS-GPS-200 Revision E, paragraph 6.3.6
    }
  { U.S. GPS satellite  }
  { was #define dname(params) para_def_expr }
  { argument types are unknown }
  { return type might be wrong }   

  function GPS_PRN(n : longint) : boolean;

  { Ground Based Augmentation System and other augmentation systems  }
  { was #define dname(params) para_def_expr }
  { argument types are unknown }
  { return type might be wrong }   
  function GBAS_PRN(n : longint) : boolean;

  { Satellite Based Augmentation System  }
  { was #define dname(params) para_def_expr }
  { argument types are unknown }
  { return type might be wrong }   
  function SBAS_PRN(n : longint) : boolean;

  { other Global Navigation Satellite System  }
  { was #define dname(params) para_def_expr }
  { argument types are unknown }
  { return type might be wrong }   
  function GNSS_PRN(n : longint) : boolean;


  {
   * Is an MMSI number that of an auxiliary associated with a mother ship?
   * We need to be able to test this for decoding AIS Type 24 messages.
   * According to <http://www.navcen.uscg.gov/marcomms/gmdss/mmsi.htm#format>,
   * auxiliary-craft MMSIs have the form 98MIDXXXX, where MID is a country
   * code and XXXX the vessel ID.
    }
  { was #define dname(params) para_def_expr }
  { argument types are unknown }
  { return type might be wrong }   

  function AIS_AUXILIARY_MMSI(n : longint) : boolean;



  { difference between timespecs in nanoseconds  }
  { int is too small, avoid floats   }
  { was #define dname(params) para_def_expr }
  { argument types are unknown }

  //function timespec_diff_ns(x,y : longint) : longint;  

  { was #define dname(params) para_def_expr }
  { argument types are unknown }
  { return type might be wrong }   

  function BAD_SOCKET(s : longint) : boolean;

  {#define INVALIDATE_SOCKET(s)	s = -1 }
  { mode flags for setting streaming policy  }
  { enable streaming  }
  const
    WATCH_ENABLE = $000001;    
  { disable watching  }
    WATCH_DISABLE = $000002;    
  { JSON output  }
    WATCH_JSON = $000010;    
  { output in NMEA  }
    WATCH_NMEA = $000020;    
  { output of packets in hex  }
    WATCH_RARE = $000040;    
  { output of raw packets  }
    WATCH_RAW = $000080;    
  { scale output to floats  }
    WATCH_SCALED = $000100;    
  { timing information  }
    WATCH_TIMING = $000200;    
  { watch specific device  }
    WATCH_DEVICE = $000800;    
  { split AIS Type 24s  }
    WATCH_SPLIT24 = $001000;    
  { enable PPS JSON  }
    WATCH_PPS = $002000;    
  { force JSON streaming  }
    WATCH_NEWSTYLE = $010000;    
  { force old-style streaming  }
    WATCH_OLDSTYLE = $020000;    
  {
   * Main structure that includes all previous substructures
    }
  ONLINE_SET	= UINT64(1) shl 1;
  TIME_SET	= UINT64(1) shl 2;
  TIMERR_SET	= UINT64(1) shl 3;
  LATLON_SET	= UINT64(1) shl 4;
  ALTITUDE_SET	= UINT64(1) shl 5;
  SPEED_SET	= UINT64(1) shl 6;
  TRACK_SET	= UINT64(1) shl 7;
  CLIMB_SET	= UINT64(1) shl 8;
  STATUS_SET	= UINT64(1) shl 9;
  MODE_SET	= UINT64(1) shl 10;
  DOP_SET  	= UINT64(1) shl 11;
  HERR_SET	= UINT64(1) shl 12;
  VERR_SET	= UINT64(1) shl 13;
  ATTITUDE_SET	= UINT64(1) shl 14;
  SATELLITE_SET	= UINT64(1) shl 15;
  SPEEDERR_SET	= UINT64(1) shl 16;
  TRACKERR_SET	= UINT64(1) shl 17;
  CLIMBERR_SET	= UINT64(1) shl 18;
  DEVICE_SET	= UINT64(1) shl 19;
  DEVICELIST_SET	= UINT64(1) shl 20;
  DEVICEID_SET	= UINT64(1) shl 21;
  RTCM2_SET	= UINT64(1) shl 22;
  RTCM3_SET	= UINT64(1) shl 23;
  AIS_SET 	= UINT64(1) shl 24;
  PACKET_SET	= UINT64(1) shl 25;
  SUBFRAME_SET	= UINT64(1) shl 26;
  GST_SET 	= UINT64(1) shl 27;
  VERSION_SET	= UINT64(1) shl 28;
  POLICY_SET	= UINT64(1) shl 29;
  LOGMESSAGE_SET	= UINT64(1) shl 30;
  ERROR_SET	= UINT64(1) shl 31;
  TIMEDRIFT_SET	= UINT64(1) shl 32;
  EOF_SET		= UINT64(1) shl 33;
  SET_HIGH_BIT	= 34;

  { no  }
    STATUS_NO_FIX = 0;    
  { yes, without DGPS  }
    STATUS_FIX = 1;    
  { yes, with DGPS  }
    STATUS_DGPS_FIX = 2;    
    UNION_SET = (((((((((RTCM2_SET or RTCM3_SET) or SUBFRAME_SET) or AIS_SET) or ATTITUDE_SET) or GST_SET) or VERSION_SET) or DEVICELIST_SET) or LOGMESSAGE_SET) or ERROR_SET) or TIMEDRIFT_SET;    

  {@null@ }(* Const before type ignored *)
  {@null@ }(* Const before type ignored *)
  {@out@ }
  function gps_open(_para1:Pchar; _para2:Pchar; _para3:Pgps_data_t):longint;cdecl; external gpslib;

  function gps_close(_para1:Pgps_data_t):longint;cdecl; external gpslib;

(* Const before type ignored *)
 // function gps_send(_para1:Pgps_data_t; _para2:Pchar; args:array of const):longint;cdecl; overload;

  function gps_send(_para1:Pgps_data_t; _para2:Pchar):longint;cdecl; external gpslib;

  {@out@ }  function gps_read(_para1:Pgps_data_t):longint;cdecl; external gpslib;

  function gps_unpack(_para1:Pchar; _para2:Pgps_data_t):longint;cdecl; external gpslib;

(* Const before type ignored *)
  function gps_waiting(_para1:Pgps_data_t; _para2:longint):boolean;cdecl; external gpslib;

  {@null@ }  function gps_stream(_para1:Pgps_data_t; _para2:dword; _para3:pointer):longint;cdecl; external gpslib;
 type
  TPgps_datat = procedure (_para1:Pgps_data_t); cdecl;
  function gps_mainloop(_para1:Pgps_data_t; _para2:longint; _para3:TPgps_datat):longint;cdecl; external gpslib;

(* Const before type ignored *)
  {@null observer@ }(* Const before type ignored *)
  function gps_data(_para1:Pgps_data_t):Pchar;cdecl; external gpslib;

(* Const before type ignored *)
  {@observer@ }(* Const before type ignored *)
  function gps_errstr(_para1:longint):Pchar;cdecl; external gpslib;

(* Const before type ignored *)
  {@null@ }(* Const before type ignored *)
  function json_pps_read(buf:Pchar; _para2:Pgps_data_t; _para3:PPchar):longint;

  { dependencies on struct gpsdata_t end hrere  }
(* Const before type ignored *)
  //procedure libgps_trace(errlevel:longint; _para2:Pchar; args:array of const);cdecl;

  procedure libgps_trace(errlevel:longint; _para2:Pchar);cdecl; external gpslib;

  {@ out @ }  procedure gps_clear_fix(_para1:Pgps_fix_t);cdecl; external gpslib;

  {@out@ }  procedure gps_clear_dop(_para1:Pdop_t);cdecl; external gpslib;

  {@ out @ }  {@ in @ }  procedure gps_merge_fix(_para1:Pgps_fix_t; _para2:gps_mask_t; _para3:Pgps_fix_t);cdecl; external gpslib;

  procedure gps_enable_debug(_para1:longint; _para2:PFILE);cdecl; external gpslib;

  {@observer@ }(* Const before type ignored *)
  function gps_maskdump(_para1:gps_mask_t):Pchar;cdecl; external gpslib;

(* Const before type ignored *)
  function safe_atof(_para1:Pchar):double;cdecl; external gpslib;

  {extern time_t mkgmtime(register struct tm *); }
  function timestamp:timestamp_t;cdecl; external gpslib;

  function iso8601_to_unix(_para1:Pchar):timestamp_t;cdecl; external gpslib;

  {extern /*@observer@*/char *unix_to_iso8601(timestamp_t t, /*@ out @*/char[], size_t len); }
  function earth_distance(_para1:double; _para2:double; _para3:double; _para4:double):double;cdecl; external gpslib;

  {@null@ }  {@out@ }  {@null@ }  {@out@ }  function earth_distance_and_bearings(_para1:double; _para2:double; _para3:double; _para4:double; _para5:Pdouble;
             _para6:Pdouble):double;cdecl; external gpslib;

  function wgs84_separation(_para1:double; _para2:double):double;cdecl; external gpslib;

  { some multipliers for interpreting GPS output  }
  { Meters to U.S./British feet  }
  const
    METERS_TO_FEET = 3.2808399;    
  { Meters to miles  }
    METERS_TO_MILES = 0.00062137119;    
  { Meters to fathoms  }
    METERS_TO_FATHOMS = 0.54680665;    
  { Knots to miles per hour  }
    KNOTS_TO_MPH = 1.1507794;    
  { Knots to kilometers per hour  }
    KNOTS_TO_KPH = 1.852;    
  { Knots to meters per second  }
    KNOTS_TO_MPS = 0.51444444;    
  { Meters per second to klicks/hr  }
    MPS_TO_KPH = 3.6;    
  { Meters/second to miles per hour  }
    MPS_TO_MPH = 2.2369363;    
  { Meters per second to knots  }
    MPS_TO_KNOTS = 1.9438445;    
  { miles and knots are both the international standard versions of the units  }
  { angle conversion multipliers  }
    GPS_PI = 3.1415926535897932384626433832795029;    
    RAD_2_DEG = 57.2957795130823208767981548141051703;    
    DEG_2_RAD = 0.0174532925199432957692369076848861271;    
  { geodetic constants  }
  { equatorial radius  }
    WGS84A = 6378137;    
  { flattening  }
    WGS84F = 298.257223563;    
  { polar radius  }
    WGS84B = 6356752.3142;    
  { netlib_connectsock() errno return values  }
  { can't get service entry  }
    NL_NOSERVICE = -(1);    
  { can't get host entry  }
    NL_NOHOST = -(2);    
  { can't get protocol entry  }
    NL_NOPROTO = -(3);    
  { can't create socket  }
    NL_NOSOCK = -(4);    
  { error SETSOCKOPT SO_REUSEADDR  }
    NL_NOSOCKOPT = -(5);    
  { can't connect to host/socket pair  }
    NL_NOCONNECT = -(6);    
  { shared-memory segment not available  }
    SHM_NOSHARED = -(7);    
  { shared-memory attach failed  }
    SHM_NOATTACH = -(8);    
  { DBUS initialization failure  }
    DBUS_FAILURE = -(9);    
  { IANA assignment  }
    DEFAULT_GPSD_PORT = '2947';    
  { IANA assignment  }
    DEFAULT_RTCM_PORT = '2101';    
  { special host values for non-socket exports  }
    GPSD_SHARED_MEMORY = 'shared memory';    
    GPSD_DBUS_EXPORT = 'DBUS export';    
  {
   * Platform-specific declarations
    }
  { Some libcs don't have strlcat/strlcpy. Local copies are provided  }
  {@out@ }  {@in@ }(* Const before type ignored *)

  function strlcat(dst:Pchar; src:Pchar; size:size_t):size_t;

  {@out@ }  {@in@ }(* Const before type ignored *)
  function strlcpy(dst:Pchar; src:Pchar; size:size_t):size_t;

  { gps.h ends here  }

implementation

  { was #define dname(params) para_def_expr }
  { argument types are unknown }
  { return type might be wrong }   
  function GPS_PRN(n : longint) : boolean;
  begin
    GPS_PRN:=(n>=1) and (n<=63);
  end;

  { was #define dname(params) para_def_expr }
  { argument types are unknown }
  { return type might be wrong }   
  function GBAS_PRN(n : longint) : boolean;
  begin
    GBAS_PRN:= (n>=64) and (n<=119);
  end;

  { was #define dname(params) para_def_expr }
  { argument types are unknown }
  { return type might be wrong }   
  function SBAS_PRN(n : longint) : boolean;
  begin
    SBAS_PRN:= (n>=120) and (n<=158);
  end;

  { was #define dname(params) para_def_expr }
  { argument types are unknown }
  { return type might be wrong }   
  function GNSS_PRN(n : longint) : boolean;
  begin
    GNSS_PRN:=(n>=159) and (n<=210);
  end;

  { was #define dname(params) para_def_expr }
  { argument types are unknown }
  { return type might be wrong }   
  function AIS_AUXILIARY_MMSI(n : longint) : boolean;
  begin
    AIS_AUXILIARY_MMSI:=(n/10000000)=98;
  end;

  { was #define dname(params) para_def_expr }
  { argument types are unknown }
  (*
  function timespec_diff_ns(x,y : longint) : longint;
  begin
    timespec_diff_ns:=longint(((((x.tv_sec)-(y.tv_sec))*1000000000)+(x.tv_nsec))-(y.tv_nsec));
  end;
  *)
  { was #define dname(params) para_def_expr }
  { argument types are unknown }
  { return type might be wrong }   
  function BAD_SOCKET(s : longint) : boolean;
  begin
    BAD_SOCKET:=s=(-(1));
  end;

  function json_pps_read(buf:Pchar; _para2:Pgps_data_t; _para3:PPchar):longint;
  begin
    { You must implement this function }
  end;
  function strlcat(dst:Pchar; src:Pchar; size:size_t):size_t;
  begin
    { You must implement this function }
  end;
  function strlcpy(dst:Pchar; src:Pchar; size:size_t):size_t;
  begin
    { You must implement this function }
  end;

end.
