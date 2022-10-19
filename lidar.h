const int N_DATA_QUADS = 4;                // there are 4 groups of data elements
const int N_ELEMENTS_PER_QUAD = 4;         // viz., 0=distance LSB; 1=distance MSB; 2=sig LSB; 3=sig MSB

// Offsets to bytes within 'Packet'
const int OFFSET_TO_START = 0;
const int OFFSET_TO_INDEX = OFFSET_TO_START + 1;
const int OFFSET_TO_SPEED_LSB = OFFSET_TO_INDEX + 1;
const int OFFSET_TO_SPEED_MSB = OFFSET_TO_SPEED_LSB + 1;
const int OFFSET_TO_4_DATA_READINGS = OFFSET_TO_SPEED_MSB + 1;
const int OFFSET_TO_CRC_L = OFFSET_TO_4_DATA_READINGS + (N_DATA_QUADS * N_ELEMENTS_PER_QUAD);
const int OFFSET_TO_CRC_M = OFFSET_TO_CRC_L + 1;
const int PACKET_LENGTH = OFFSET_TO_CRC_M + 1;  // length of a complete packet
// Offsets to the (4) elements of each of the (4) data quads
const int OFFSET_DATA_DISTANCE_LSB = 0;
const int OFFSET_DATA_DISTANCE_MSB = OFFSET_DATA_DISTANCE_LSB + 1;
const int OFFSET_DATA_SIGNAL_LSB = OFFSET_DATA_DISTANCE_MSB + 1;
const int OFFSET_DATA_SIGNAL_MSB = OFFSET_DATA_SIGNAL_LSB + 1;

int Packet[PACKET_LENGTH];                 // an input packet
int ixPacket = 0;                          // index into 'Packet' array
const int VALID_PACKET = 0;
const int INVALID_PACKET = VALID_PACKET + 1;
const byte INVALID_DATA_FLAG = (1 << 7);   // Mask for byte 1 of each data quad "Invalid data"

uint8_t motor_rph_high_byte = 0;
uint8_t motor_rph_low_byte = 0;
uint16_t motor_rph = 0;

uint8_t inByte = 0;  // incoming serial byte

const unsigned char COMMAND = 0xFA;        // Start of new packet
const int INDEX_LO = 0xA0;                 // lowest index value
const int INDEX_HI = 0xF9;                 // highest index value

uint16_t startingAngle = 0;                      // the first scan angle (of group of 4, based on 'index'), in degrees (0..359)

#define PWM_FREQ 32768/2
double pwm_val = 500;          // start with ~50% power
double pwm_last;
double motor_rpm;

uint16_t aryDist[N_DATA_QUADS] = {0, 0, 0, 0};   // thre are (4) distances, one for each data quad
// so the maximum distance is 16383 mm (0x3FFF)
uint16_t aryQuality[N_DATA_QUADS] = {0, 0, 0, 0}; // same with 'quality'

/* REF: https://github.com/Xevel/NXV11/wiki
  The bit 7 of byte 1 seems to indicate that the distance could not be calculated.
  It's interesting to see that when this bit is set, the second byte is always 80, and the values of the first byte seem to be
  only 02, 03, 21, 25, 35 or 50... When it's 21, then the whole block is 21 80 XX XX, but for all the other values it's the
  data block is YY 80 00 00 maybe it's a code to say what type of error ? (35 is preponderant, 21 seems to be when the beam is
  interrupted by the supports of the cover) .
*/
const byte STRENGTH_WARNING_FLAG = (1 << 6);  // Mask for byte 1 of each data quat "Strength Warning"

/*
  The bit 6 of byte 1 is a warning when the reported strength is greatly inferior to what is expected at this distance.
  This may happen when the material has a low reflectance (black material...), or when the dot does not have the expected
  size or shape (porous material, transparent fabric, grid, edge of an object...), or maybe when there are parasitic
  reflections (glass... ).
*/
const byte BAD_DATA_MASK = (INVALID_DATA_FLAG | STRENGTH_WARNING_FLAG);

unsigned long now;
unsigned long motor_check_timer = millis();
unsigned long motor_check_interval = 200;
unsigned int rpm_err_thresh = 10;  // 2 seconds (10 * 200ms) to shutdown motor with improper RPM and high voltage
unsigned int rpm_err = 0;
