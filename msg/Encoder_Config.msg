int8 CANCoder=0

int8 ENCODERSOURCE_RELATIVE=0
int8 ENCODERSOURCE_ABSOLUTE=1

int8 CAN_RIO_INTERNAL=0
int8 CAN_RIO_CANIVORE=1
int8 CAN_COPROCESSOR_CAN=2

int8 UNUSED_ENCODER=0
int8 ABSOLUTE_SEED_ENCODER=1
int8 FULL_USE_ENCODER=2

int8 BOOT_TO_ZERO=0
int8 BOOT_TO_ABS_VALUE=1

int32 id
int8 encoder_type
int8 sensor_source
int8 can_network
int8 init_strategy
int8 control_mode
float64 magnetic_offset