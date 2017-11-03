#define Light_OFF       0x01      // Command to turn the light off
#define Light_ON_50     0x02      // Command to turn the light on 50%
#define Light_ON_100    0x03      // Command to turn the light on 100%
#define Light_PAIR      0xFF      // Command to pair a remote to the light

void initAnsluta();
void ReadAddressBytes();
void SendCommand(byte AddressByteA, byte AddressByteB, byte Command);