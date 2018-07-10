#include "devices/src/gps_helper.h" 
#include "definitions.h"

#include <uORB/topics/actuator_armed.h>

/* Decoder state */
typedef enum {
	RST_RTK_SWARM_DECODE_SYNC1 = 0,
    RST_RTK_SWARM_DECODE_SYNC2,
    RST_RTK_SWARM_DECODE_CLASS,
    RST_RTK_SWARM_DECODE_ID,
    RST_RTK_SWARM_DECODE_LENGTH1,
    RST_RTK_SWARM_DECODE_LENGTH2, 
	RST_RTK_SWARM_DECODE_PAYLOAD,
    RST_RTK_SWARM_DECODE_CHKSUM1,
    RST_RTK_SWARM_DECODE_CHKSUM2
} rst_rtk_swarm_decode_state_t;

typedef struct {
	uint32_t	iTOW;		/**< GPS Time of Week [ms] */
	uint16_t	year; 		/**< Year (UTC)*/
	uint8_t		month; 		/**< Month, range 1..12 (UTC) */
	uint8_t		day; 		/**< Day of month, range 1..31 (UTC) */
	uint8_t		hour; 		/**< Hour of day, range 0..23 (UTC) */
	uint8_t		min; 		/**< Minute of hour, range 0..59 (UTC) */
	uint8_t		sec;		/**< Seconds of minute, range 0..60 (UTC) */
	uint8_t		valid; 		/**< Validity flags (see UBX_RX_NAV_PVT_VALID_...) */
	uint32_t	tAcc; 		/**< Time accuracy estimate (UTC) [ns] */
	int32_t		nano;		/**< Fraction of second (UTC) [-1e9...1e9 ns] */
	uint8_t		fixType;	/**< GNSSfix type: 0 = No fix, 1 = Dead Reckoning only, 2 = 2D fix, 3 = 3d-fix, 4 = GNSS + dead reckoning, 5 = time only fix */
	uint8_t		flags;		/**< Fix Status Flags (see UBX_RX_NAV_PVT_FLAGS_...) */
	uint8_t		flags2;
	uint8_t		numsv;		/**< Number of SVs used in Nav Solution */
	int32_t		lon;		/**< Longitude [1e-7 deg] */
	int32_t		lat;		/**< Latitude [1e-7 deg] */
	int32_t		height;		/**< Height above ellipsoid [mm] */
	int32_t		hMSL;		/**< Height above mean sea level [mm] */
	uint32_t	hAcc;  		/**< Horizontal accuracy estimate [mm] */
	uint32_t	vAcc;  		/**< Vertical accuracy estimate [mm] */
	int32_t		velN;		/**< NED north velocity [mm/s]*/
	int32_t		velE;		/**< NED east velocity [mm/s]*/
	int32_t		velD;		/**< NED down velocity [mm/s]*/
	int32_t		gSpeed;		/**< Ground Speed (2-D) [mm/s] */
	int32_t		headMot;	/**< Heading of motion (2-D) [1e-5 deg] */
	uint32_t	sAcc;		/**< Speed accuracy estimate [mm/s] */
	uint32_t	headAcc;	/**< Heading accuracy estimate (motion and vehicle) [1e-5 deg] */
	uint16_t	pDOP;		/**< Position DOP [0.01] */
    uint8_t	    reserved1;
    uint8_t	    reserved2;
    uint8_t	    reserved3;
    uint8_t	    reserved4;
    uint8_t	    reserved5;
    uint8_t	    reserved6;
	int32_t		headVeh;	/**< (ubx8+ only) Heading of vehicle (2-D) [1e-5 deg] */
    uint8_t	    reserved7;	/**< (ubx8+ only) */
    uint8_t	    reserved8;
    uint8_t	    reserved9;
    uint8_t	    reserved10;
    

} rst_rtk_swarm_t;                          /**使用中海达RTK*/

class GPSDriverRST_RTK_SWARM : public GPSHelper
{
public:
	GPSDriverRST_RTK_SWARM(Interface gpsInterface, GPSCallbackPtr callback, void *callback_user,
		     struct vehicle_gps_position_s *gps_position);
             
    virtual ~GPSDriverRST_RTK_SWARM();

    int receive(unsigned timeout) override;
    int configure(unsigned &baudrate, OutputMode output_mode) override;

    int armed_sub = orb_subscribe(ORB_ID(actuator_armed));
    struct actuator_armed_s armed;
    
private:
    int parseChar(const uint8_t b);

    void decodeInit(void);

    int payloadRxAdd(const uint8_t b);

    int payloadRxDone(void);

    void addByteToChecksum(const uint8_t);

    struct vehicle_gps_position_s *_gps_position {nullptr};

    uint16_t		_rx_payload_index{};
    uint8_t		_buf[200];
    rst_rtk_swarm_t rtk_swarm_data{};

    rst_rtk_swarm_decode_state_t	_decode_state{};

    const Interface		_interface;

    bool			_configured{false};

    OutputMode		_output_mode{OutputMode::GPS};

    bool			_got_posllh{false};
    bool			_got_velned{false};
    
    uint16_t Checksum;

    uint16_t		_rx_payload_length{};

    uint64_t		_last_timestamp_time{0};


    uint8_t         _CLASS;
    uint16_t         _ID;
    uint16_t        _Length;
    uint8_t			_rx_ck_a{};
	uint8_t			_rx_ck_b{};
};

