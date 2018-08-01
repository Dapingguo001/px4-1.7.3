#include <assert.h>
#include <math.h>
#include <stdio.h>
#include <string.h>
#include <ctime>
#include <systemlib/param/param.h>

#include <uORB/topics/actuator_armed.h>

#include "rst_rtk_swarm.h"

#include "devices/src/gps_helper.h" 

/**** Warning macros, disable to save memory */
#define RST_RTK_WARN(...)		{GPS_WARN(__VA_ARGS__);}
#define RST_RTK_DEBUG(...)		{/*GPS_WARN(__VA_ARGS__);*/}

#define RST_CAN_PACKET_TIMEOUT	2		// ms, if now data during this delay assume that full update received

GPSDriverRST_RTK_SWARM::GPSDriverRST_RTK_SWARM(Interface gpsInterface, GPSCallbackPtr callback, void *callback_user,
    struct vehicle_gps_position_s *gps_position)
    : GPSHelper(callback, callback_user)
    , _gps_position(gps_position)
    , _interface(gpsInterface)
{
    decodeInit();
	memset(&armed, 0, sizeof(armed));
}

GPSDriverRST_RTK_SWARM::~GPSDriverRST_RTK_SWARM()
{

}

int
GPSDriverRST_RTK_SWARM::configure(unsigned &baudrate, OutputMode output_mode)
{
    _configured = false;
    _output_mode = output_mode;
    
    const unsigned const_baudrate = 115200;

    if (_interface == Interface::UART) {
        setBaudrate(const_baudrate);
        /* flush input and wait for at least 20 ms silence */
		decodeInit();
		receive(20);
		decodeInit();
    }
    _configured = true;
	return 0;
}



int	// -1 = error, 0 = no message handled, 1 = message handled, 2 = sat info message handled
GPSDriverRST_RTK_SWARM::receive(unsigned timeout)
{
	uint8_t buf[GPS_READ_BUFFER_SIZE * 2];
	/* timeout additional to poll */
	gps_abstime time_started = gps_absolute_time();

    int handled = 0;

	while (true) {
		bool ready_to_return = _configured ? (_got_posllh && _got_velned) : handled;

		/* Wait for only RST_CAN_PACKET_TIMEOUT if something already received. */
        int ret = read(buf, sizeof(buf), ready_to_return ? RST_CAN_PACKET_TIMEOUT : timeout);

		if (ret < 0) {
			/* something went wrong when polling or reading */
			RST_RTK_WARN("ubx poll_or_read err");
			return -1;

		} else if (ret == 0) {
			/* return success if ready */
			if (ready_to_return) {
				_got_posllh = false;
                _got_velned = false;
				return handled;
			}

		} else {
			//RST_RTK_DEBUG("read %d bytes", ret);
            /* pass received bytes to the packet decoder */
			for (int i = 0; i < ret; i++) {
				handled |= parseChar(buf[i]);
			}
		}

		/* abort after timeout if no useful packets received */
		if (time_started + timeout * 1000 < gps_absolute_time()) {
			RST_RTK_DEBUG("timed out, returning");
			return -1;
		}
	}
}

int	// 0 = decoding, 1 = message handled, 2 = sat info message handled
GPSDriverRST_RTK_SWARM::parseChar(const uint8_t b)
{
    int ret = 0;
	switch (_decode_state) {
        case RST_RTK_SWARM_DECODE_SYNC1:
            if(b == 0xB5){
                _decode_state = RST_RTK_SWARM_DECODE_SYNC2;
            }

            break;

        case RST_RTK_SWARM_DECODE_SYNC2:
            if(b == 0x62){
                _decode_state = RST_RTK_SWARM_DECODE_CLASS;
            }
            else{
                decodeInit();
                _decode_state = RST_RTK_SWARM_DECODE_SYNC1;
            }

            break;

        case RST_RTK_SWARM_DECODE_CLASS:
            _CLASS = b;
            addByteToChecksum(b);
            _decode_state = RST_RTK_SWARM_DECODE_ID;
            break;

        case RST_RTK_SWARM_DECODE_ID:
            _ID = (uint16_t)(_CLASS | b<<8);
            addByteToChecksum(b);
            _decode_state = RST_RTK_SWARM_DECODE_LENGTH1;
            break;
            
        case RST_RTK_SWARM_DECODE_LENGTH1:
            _Length = b;
            addByteToChecksum(b);
            _decode_state = RST_RTK_SWARM_DECODE_LENGTH2;
            break;

        case RST_RTK_SWARM_DECODE_LENGTH2:
            _Length = (_Length | b<<8);
            addByteToChecksum(b);
            _decode_state = RST_RTK_SWARM_DECODE_PAYLOAD;
            break;        

        case RST_RTK_SWARM_DECODE_PAYLOAD:
            ret = payloadRxAdd(b);
            addByteToChecksum(b);
            if(ret == 1){
                _decode_state = RST_RTK_SWARM_DECODE_CHKSUM1;
                ret = 0;
            }
            break;

        case RST_RTK_SWARM_DECODE_CHKSUM1:
            if(_rx_ck_a == b)
            {
                Checksum = (uint16_t)((uint16_t)(b) << 8);
                _decode_state = RST_RTK_SWARM_DECODE_CHKSUM2;
            }
            else
            {
                _decode_state = RST_RTK_SWARM_DECODE_SYNC1;
                decodeInit();
            }
            break;

        case RST_RTK_SWARM_DECODE_CHKSUM2:
            if(_rx_ck_b == b)
            {
                ret = payloadRxDone();	// finish payload processing
            }
            else
            {
                RST_RTK_DEBUG("rst rtk checksum err");
            }
            decodeInit();
            break;
        
        default:
            break;
    }
    return ret;
}

void
GPSDriverRST_RTK_SWARM::addByteToChecksum(const uint8_t b)
{
    _rx_ck_a = _rx_ck_a + b;
	_rx_ck_b = _rx_ck_b + _rx_ck_a;
}

/**
 * Add payload rx byte
 */
int	// -1 = error, 0 = ok, 1 = payload completed
GPSDriverRST_RTK_SWARM::payloadRxAdd(const uint8_t b)
{
	int ret = 0;
	uint8_t *p_buf = (uint8_t *)&_buf;

	p_buf[_rx_payload_index] = b;

	if (++_rx_payload_index >= _Length) {
		ret = 1;	// payload received completely
	}

	return ret;
}


int	// 0 = no message handled, 1 = message handled, 2 = sat info message handled
GPSDriverRST_RTK_SWARM::payloadRxDone()
{
    bool updated;
    //缓冲区中获取数据
    memcpy((&rtk_swarm_data.iTOW), _buf, 4);              
    memcpy((&rtk_swarm_data.year) ,_buf + 4, 2);
    memcpy((&rtk_swarm_data.month) ,_buf + 6, 1);
    memcpy((&rtk_swarm_data.day) ,_buf + 7,  1);
    memcpy((&rtk_swarm_data.hour) ,_buf + 8,  1);
    memcpy((&rtk_swarm_data.min) ,_buf + 9,  1);   
    memcpy((&rtk_swarm_data.sec) ,_buf + 10,  1);
    memcpy((&rtk_swarm_data.valid) ,_buf + 11,  1);
    memcpy((&rtk_swarm_data.tAcc) ,_buf + 12,  4);
    memcpy((&rtk_swarm_data.nano) ,_buf + 16,  4);
    memcpy((&rtk_swarm_data.fixType) ,_buf + 20,  1);
    memcpy((&rtk_swarm_data.flags) ,_buf + 21,  1);
    memcpy((&rtk_swarm_data.flags2) ,_buf + 22,  1);
    memcpy((&rtk_swarm_data.numsv) ,_buf + 23,  1);
    memcpy((&rtk_swarm_data.lon) ,_buf + 24,  4);
    memcpy((&rtk_swarm_data.lat) ,_buf + 28,  4);
    memcpy((&rtk_swarm_data.height) ,_buf + 32,  4);
    memcpy((&rtk_swarm_data.hMSL) ,_buf + 36,  4);
    memcpy((&rtk_swarm_data.hAcc) ,_buf + 40,  4);
    memcpy((&rtk_swarm_data.vAcc) ,_buf + 44,  4);
    memcpy((&rtk_swarm_data.velN) ,_buf + 48,  4);
    memcpy((&rtk_swarm_data.velE) ,_buf + 52,  4); 
    memcpy((&rtk_swarm_data.velD) ,_buf + 56,  4);  
    memcpy((&rtk_swarm_data.gSpeed) ,_buf + 60,  4);
    memcpy((&rtk_swarm_data.headMot) ,_buf + 64,  4);
    memcpy((&rtk_swarm_data.sAcc) ,_buf + 68,  4); 
    memcpy((&rtk_swarm_data.headAcc) ,_buf + 72,  4); 
    memcpy((&rtk_swarm_data.pDOP) ,_buf + 76,  2); 
    memcpy((&rtk_swarm_data.reserved1) ,_buf + 78,  1); 
    memcpy((&rtk_swarm_data.reserved2) ,_buf + 79,  1);
    memcpy((&rtk_swarm_data.reserved3) ,_buf + 80,  1);
    memcpy((&rtk_swarm_data.reserved4) ,_buf + 81,  1);
    memcpy((&rtk_swarm_data.reserved5) ,_buf + 82,  1);
    memcpy((&rtk_swarm_data.reserved6) ,_buf + 83,  1);     
    memcpy((&rtk_swarm_data.headVeh) ,_buf + 84,  4);
    memcpy((&rtk_swarm_data.reserved7) ,_buf + 88,  1);   
    memcpy((&rtk_swarm_data.reserved8) ,_buf + 89,  1);   
    memcpy((&rtk_swarm_data.reserved9) ,_buf + 90,  1);   
    memcpy((&rtk_swarm_data.reserved10) ,_buf + 91,  1);

    /**判断GPS定位类型*/
    /***Fix_type>3 GPS有效*/
    orb_check(armed_sub, &updated);
    
    if (updated) {
            orb_copy(ORB_ID(actuator_armed), armed_sub, &armed);
    }

//    if(armed.armed){
        if(((rtk_swarm_data.flags >> 6) & 0x03) == 2){
            _gps_position->fix_type =  6;         //fixed RTK
            _gps_position->vel_ned_valid = true;  //
            
        }
        else if(((rtk_swarm_data.flags >> 6) & 0x03) == 1){ //float RTK
            _gps_position->fix_type =  5;
            _gps_position->vel_ned_valid = true;
        }
        else if ((rtk_swarm_data.flags & 0x01) == 1){  //DGPS
            _gps_position->fix_type =  4;          
            _gps_position->vel_ned_valid = true;
        }
        else{
            _gps_position->fix_type =  0;
            _gps_position->vel_ned_valid = false;  
        }
//    }
/*    else{
        if(rtk_swarm_data.fixType == 0){    
            _gps_position->fix_type =  0;
            _gps_position->vel_ned_valid = false;
        }
        else if(rtk_swarm_data.fixType == 3){
            _gps_position->fix_type =  0;         //3D fix
            _gps_position->vel_ned_valid = false;  //
        }
        else if((rtk_swarm_data.flags & 0x01) == 1)
        {
            _gps_position->fix_type =  0;         //DGPS
            _gps_position->vel_ned_valid = false;  //
        }
        else if (((rtk_swarm_data.flags >> 6) & 0x03) == 2){        //fixed RTK
            _gps_position->fix_type =  6;          
            _gps_position->vel_ned_valid = true;
        }
        else if(((rtk_swarm_data.flags >> 6) & 0x03) == 1){           //float RTK
            _gps_position->fix_type =  0;
            _gps_position->vel_ned_valid = false;
        }
        else{
            _gps_position->fix_type =  0;
            _gps_position->vel_ned_valid = false;  
        }
    }*/

    _gps_position->satellites_used = rtk_swarm_data.numsv;
    _gps_position->lat = rtk_swarm_data.lat;
    _gps_position->lon = rtk_swarm_data.lon;
    _gps_position->alt = rtk_swarm_data.hMSL;
    _gps_position->alt_ellipsoid = rtk_swarm_data.height;

    _gps_position->eph = 0.8f;//(float)(rtk_swarm_data.hAcc * 1e-3f);
    _gps_position->epv = 1.2f;//(float)(rtk_swarm_data.vAcc * 1e-3f);
    _gps_position->s_variance_m_s = 0.2f;//(float)(rtk_swarm_data.sAcc * 1e-3f);
    _gps_position->vel_m_s = (float)(rtk_swarm_data.gSpeed * 1e-3f);

    _gps_position->vel_n_m_s = (float)(rtk_swarm_data.velN * 1e-3f);
    _gps_position->vel_e_m_s = (float)(rtk_swarm_data.velE * 1e-3f);
    _gps_position->vel_d_m_s = (float)(rtk_swarm_data.velD * 1e-3f);

    _gps_position->cog_rad = (float)(rtk_swarm_data.headMot * M_DEG_TO_RAD_F * 1e-5f);   //运动航向（实际运动方向，不是机头航向）
    _gps_position->c_variance_rad =  (float)(rtk_swarm_data.headAcc * M_DEG_TO_RAD_F * 1e-5f);            //航向精度估计（无人机实际运动方向），预测值

	/* convert to unix timestamp */
    struct tm timeinfo;
    memset(&timeinfo, 0, sizeof(timeinfo));
    timeinfo.tm_year	=  rtk_swarm_data.year - 1900;
    timeinfo.tm_mon		=  rtk_swarm_data.month - 1;
    timeinfo.tm_mday	=  rtk_swarm_data.day;
    timeinfo.tm_hour	=  rtk_swarm_data.hour;
    timeinfo.tm_min		=  rtk_swarm_data.min;
    timeinfo.tm_sec		=  rtk_swarm_data.sec;


#ifndef NO_MKTIME
    time_t epoch = mktime(&timeinfo);

    if (epoch > GPS_EPOCH_SECS) {
        // FMUv2+ boards have a hardware RTC, but GPS helps us to configure it
        // and control its drift. Since we rely on the HRT for our monotonic
        // clock, updating it from time to time is safe.

        timespec ts;
        memset(&ts, 0, sizeof(ts));
        ts.tv_sec = epoch;
        ts.tv_nsec = rtk_swarm_data.nano;

        setClock(ts);

        _gps_position->time_utc_usec = static_cast<uint64_t>(epoch) * 1000000ULL;
        _gps_position->time_utc_usec += rtk_swarm_data.nano / 1000;

    } else {
        _gps_position->time_utc_usec = 0;
    }

#else
    _gps_position->time_utc_usec = 0;
#endif


    _gps_position->timestamp = gps_absolute_time();

  _gps_position->time_utc_usec =  _gps_position->timestamp;

    _last_timestamp_time = _gps_position->timestamp;

    

    _rate_count_vel++;
    _rate_count_lat_lon++;

    _got_posllh = true;
    _got_velned = true;

    _gps_position->timestamp_time_relative = (int32_t)(_last_timestamp_time - _gps_position->timestamp);
	return true;

}

void
GPSDriverRST_RTK_SWARM::decodeInit()
{
    _decode_state = RST_RTK_SWARM_DECODE_SYNC1;
    _rx_ck_a = 0;
    _rx_ck_b = 0;
	_rx_payload_length = 134;
	_rx_payload_index = 0;
}