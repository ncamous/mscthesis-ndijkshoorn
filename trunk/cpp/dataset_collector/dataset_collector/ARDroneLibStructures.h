#pragma once

typedef float               float32_t;
typedef double              flo;
#define bool_t  int32_t

// Definition des types entiers
typedef signed    __int8    int8_t;
typedef unsigned  __int8    uint8_t;
typedef signed    __int16   int16_t;
typedef unsigned  __int16   uint16_t;
typedef signed    __int32   int32_t;
typedef unsigned  __int32   uint32_t;
typedef signed    __int64   int64_t;
typedef unsigned  __int64   uint64_t;


// Define constants for gyrometers handling
typedef enum {
  GYRO_X    = 0,
  GYRO_Y    = 1,
  GYRO_Z    = 2,
  NB_GYROS  = 3
} def_gyro_t;


// Define constants for accelerometers handling
typedef enum {
  ACC_X   = 0,
  ACC_Y   = 1,
  ACC_Z   = 2,
  NB_ACCS = 3
} def_acc_t;

/**
 * \struct _velocities_t
 * \brief Velocities in float32_t format
 */
typedef struct _velocities_t {
  float32_t x;
  float32_t y;
  float32_t z;
} velocities_t;


typedef struct _navdata_option_t {
  uint16_t  tag;
  uint16_t  size;
#if defined _MSC_VER
  uint8_t   *data;
#else
  uint8_t   data[];
#endif
} navdata_option_t;


/**
 * @brief Navdata structure sent over the network.
 */
typedef struct _navdata_t {
  uint32_t    header;			/*!< Always set to NAVDATA_HEADER */
  uint32_t    ardrone_state;    /*!< Bit mask built from def_ardrone_state_mask_t */
  uint32_t    sequence;         /*!< Sequence number, incremented for each sent packet */
  bool_t      vision_defined;

  navdata_option_t  options[1];
}_ATTRIBUTE_PACKED_ navdata_t;



/*----------------------------------------------------------------------------*/
/**
 * @brief Minimal navigation data for all flights.
 */
typedef struct _navdata_demo_t {
  uint16_t    tag;					  /*!< Navdata block ('option') identifier */
  uint16_t    size;					  /*!< set this to the size of this structure */

  uint32_t    ctrl_state;             /*!< Flying state (landed, flying, hovering, etc.) defined in CTRL_STATES enum. */
  uint32_t    vbat_flying_percentage; /*!< battery voltage filtered (mV) */

  float32_t   theta;                  /*!< UAV's pitch in milli-degrees */
  float32_t   phi;                    /*!< UAV's roll  in milli-degrees */
  float32_t   psi;                    /*!< UAV's yaw   in milli-degrees */

  int32_t     altitude;               /*!< UAV's altitude in centimeters */

  float32_t   vx;                     /*!< UAV's estimated linear velocity */
  float32_t   vy;                     /*!< UAV's estimated linear velocity */
  float32_t   vz;                     /*!< UAV's estimated linear velocity */

  uint32_t    num_frames;			  /*!< streamed frame index */ // Not used -> To integrate in video stage.

  // Camera parameters compute by detection
  matrix33_t  detection_camera_rot;   /*!<  Deprecated ! Don't use ! */
  vector31_t  detection_camera_trans; /*!<  Deprecated ! Don't use ! */
  uint32_t	  detection_tag_index;    /*!<  Deprecated ! Don't use ! */

  uint32_t	  detection_camera_type; /*!<  Type of tag searched in detection */

  // Camera parameters compute by drone
  matrix33_t  drone_camera_rot;		/*!<  Deprecated ! Don't use ! */
  vector31_t  drone_camera_trans;	/*!<  Deprecated ! Don't use ! */
}_ATTRIBUTE_PACKED_ navdata_demo_t;



/*----------------------------------------------------------------------------*/
/**
 * @brief Last navdata option that *must* be included at the end of all navdata packets
 * + 6 bytes
 */
typedef struct _navdata_cks_t {
  uint16_t  tag;
  uint16_t  size;

  // Checksum for all navdatas (including options)
  uint32_t  cks;
}_ATTRIBUTE_PACKED_ navdata_cks_t;


/*----------------------------------------------------------------------------*/
/**
 * @brief Timestamp
 * + 6 bytes
 */
typedef struct _navdata_time_t {
  uint16_t  tag;
  uint16_t  size;

  uint32_t  time;  /*!< 32 bit value where the 11 most significant bits represents the seconds, and the 21 least significant bits are the microseconds. */
}_ATTRIBUTE_PACKED_ navdata_time_t;



/*----------------------------------------------------------------------------*/
/**
 * @brief Raw sensors measurements
 */
typedef struct _navdata_raw_measures_t {
  uint16_t  tag;
  uint16_t  size;

  // +12 bytes
  uint16_t  raw_accs[NB_ACCS];    // filtered accelerometers
  uint16_t  raw_gyros[NB_GYROS];  // filtered gyrometers
  uint16_t  raw_gyros_110[2];     // gyrometers  x/y 110 deg/s
  uint32_t  vbat_raw;             // battery voltage raw (mV)
  uint16_t  us_debut_echo;
  uint16_t  us_fin_echo;
  uint16_t  us_association_echo;
  uint16_t  us_distance_echo;
  uint16_t  us_courbe_temps;
  uint16_t  us_courbe_valeur;
  uint16_t  us_courbe_ref;
}_ATTRIBUTE_PACKED_ navdata_raw_measures_t;


typedef struct _navdata_phys_measures_t {
  uint16_t   tag;
  uint16_t   size;

  float32_t   accs_temp;
  uint16_t    gyro_temp;
  float32_t   phys_accs[NB_ACCS];
  float32_t   phys_gyros[NB_GYROS];
  uint32_t    alim3V3;              // 3.3volt alim [LSB]
  uint32_t    vrefEpson;            // ref volt Epson gyro [LSB]
  uint32_t    vrefIDG;              // ref volt IDG gyro [LSB]
}_ATTRIBUTE_PACKED_ navdata_phys_measures_t;


typedef struct _navdata_gyros_offsets_t {
  uint16_t   tag;
  uint16_t   size;

  float32_t offset_g[NB_GYROS];
}_ATTRIBUTE_PACKED_ navdata_gyros_offsets_t;


typedef struct _navdata_euler_angles_t {
  uint16_t   tag;
  uint16_t   size;

  float32_t   theta_a;
  float32_t   phi_a;
}_ATTRIBUTE_PACKED_ navdata_euler_angles_t;


typedef struct _navdata_references_t {
  uint16_t   tag;
  uint16_t   size;

  int32_t   ref_theta;
  int32_t   ref_phi;
  int32_t   ref_theta_I;
  int32_t   ref_phi_I;
  int32_t   ref_pitch;
  int32_t   ref_roll;
  int32_t   ref_yaw;
  int32_t   ref_psi;
}_ATTRIBUTE_PACKED_ navdata_references_t;


typedef struct _navdata_trims_t {
  uint16_t   tag;
  uint16_t   size;

  float32_t angular_rates_trim_r;
  float32_t euler_angles_trim_theta;
  float32_t euler_angles_trim_phi;
}_ATTRIBUTE_PACKED_ navdata_trims_t;

typedef struct _navdata_rc_references_t {
  uint16_t   tag;
  uint16_t   size;

  int32_t    rc_ref_pitch;
  int32_t    rc_ref_roll;
  int32_t    rc_ref_yaw;
  int32_t    rc_ref_gaz;
  int32_t    rc_ref_ag;
}_ATTRIBUTE_PACKED_ navdata_rc_references_t;


typedef struct _navdata_pwm_t {
  uint16_t   tag;
  uint16_t   size;

  uint8_t     motor1;
  uint8_t     motor2;
  uint8_t     motor3;
  uint8_t     motor4;
  uint8_t	  sat_motor1;
  uint8_t	  sat_motor2;
  uint8_t	  sat_motor3;
  uint8_t	  sat_motor4;
  int32_t     gaz_feed_forward;
  int32_t     gaz_altitude;
  float32_t   altitude_integral;
  float32_t   vz_ref;
  int32_t     u_pitch;
  int32_t     u_roll;
  int32_t     u_yaw;
  float32_t   yaw_u_I;
  int32_t     u_pitch_planif;
  int32_t     u_roll_planif;
  int32_t     u_yaw_planif;
  float32_t   u_gaz_planif;
  uint16_t    current_motor1;
  uint16_t    current_motor2;
  uint16_t    current_motor3;
  uint16_t    current_motor4;
}_ATTRIBUTE_PACKED_ navdata_pwm_t;


typedef struct _navdata_altitude_t {
  uint16_t   tag;
  uint16_t   size;

  int32_t   altitude_vision;
  float32_t altitude_vz;
  int32_t   altitude_ref;
  int32_t   altitude_raw;
}_ATTRIBUTE_PACKED_ navdata_altitude_t;


typedef struct _navdata_vision_raw_t {
  uint16_t   tag;
  uint16_t   size;

  float32_t vision_tx_raw;
  float32_t vision_ty_raw;
  float32_t vision_tz_raw;
}_ATTRIBUTE_PACKED_ navdata_vision_raw_t;


typedef struct _navdata_vision_t {
  uint16_t   tag;
  uint16_t   size;

  uint32_t   vision_state;
  int32_t    vision_misc;
  float32_t  vision_phi_trim;
  float32_t  vision_phi_ref_prop;
  float32_t  vision_theta_trim;
  float32_t  vision_theta_ref_prop;

  int32_t    new_raw_picture;
  float32_t  theta_capture;
  float32_t  phi_capture;
  float32_t  psi_capture;
  int32_t    altitude_capture;
  uint32_t   time_capture;     // time in TSECDEC format (see config.h)
  velocities_t body_v;

  float32_t  delta_phi;
  float32_t  delta_theta;
  float32_t  delta_psi;

	uint32_t  gold_defined;
	uint32_t  gold_reset;
	float32_t gold_x;
	float32_t gold_y;
}_ATTRIBUTE_PACKED_ navdata_vision_t;


typedef struct _navdata_vision_perf_t {
  uint16_t   tag;
  uint16_t   size;

  // +44 bytes
  float32_t  time_szo;
  float32_t  time_corners;
  float32_t  time_compute;
  float32_t  time_tracking;
  float32_t  time_trans;
  float32_t  time_update;
	float32_t  time_custom[NAVDATA_MAX_CUSTOM_TIME_SAVE];
}_ATTRIBUTE_PACKED_ navdata_vision_perf_t;


typedef struct _navdata_trackers_send_t {
  uint16_t   tag;
  uint16_t   size;

  int32_t locked[DEFAULT_NB_TRACKERS_WIDTH * DEFAULT_NB_TRACKERS_HEIGHT];
  screen_point_t point[DEFAULT_NB_TRACKERS_WIDTH * DEFAULT_NB_TRACKERS_HEIGHT];
}_ATTRIBUTE_PACKED_ navdata_trackers_send_t;


typedef struct _navdata_vision_detect_t {
	/* !! Change the function 'navdata_server_reset_vision_detect()' if this structure is modified !! */
  uint16_t   tag;
  uint16_t   size;

  uint32_t   nb_detected;
  uint32_t   type[NB_NAVDATA_DETECTION_RESULTS];
  uint32_t   xc[NB_NAVDATA_DETECTION_RESULTS];
  uint32_t   yc[NB_NAVDATA_DETECTION_RESULTS];
  uint32_t   width[NB_NAVDATA_DETECTION_RESULTS];
  uint32_t   height[NB_NAVDATA_DETECTION_RESULTS];
  uint32_t   dist[NB_NAVDATA_DETECTION_RESULTS];
  float32_t  orientation_angle[NB_NAVDATA_DETECTION_RESULTS];
}_ATTRIBUTE_PACKED_ navdata_vision_detect_t;

typedef struct _navdata_vision_of_t {
  uint16_t   tag;
  uint16_t   size;

  float32_t   of_dx[5];
  float32_t   of_dy[5];
}_ATTRIBUTE_PACKED_ navdata_vision_of_t;


typedef struct _navdata_watchdog_t {
  uint16_t   tag;
  uint16_t   size;

  // +4 bytes
  int32_t    watchdog;
}_ATTRIBUTE_PACKED_ navdata_watchdog_t;

typedef struct _navdata_adc_data_frame_t {
  uint16_t  tag;
  uint16_t  size;

  uint32_t  version;
  uint8_t   data_frame[32];
}_ATTRIBUTE_PACKED_ navdata_adc_data_frame_t;

typedef struct _navdata_video_stream_t {
  uint16_t  tag;
  uint16_t  size;

  uint8_t 	quant;			// quantizer reference used to encode frame [1:31]
  uint32_t	frame_size;		// frame size (bytes)
  uint32_t	frame_number;	// frame index
  uint32_t	atcmd_ref_seq;  // atmcd ref sequence number
  uint32_t	atcmd_mean_ref_gap;	// mean time between two consecutive atcmd_ref (ms)
  float32_t atcmd_var_ref_gap;
  uint32_t	atcmd_ref_quality; // estimator of atcmd link quality
}_ATTRIBUTE_PACKED_ navdata_video_stream_t;