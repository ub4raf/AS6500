# AS6500
AS6500 (TDC-GPX2 family TDC by AMS\ScioSense) communication example:

TDC is Time to Digital Converter. AS6500 is subnanosecond one-shot TDC for lidar, PET, flowmeters.

Communication by SPI wirh master MSP430. MSP430FR6989 Launchpad w custom boosterpack-like TDC 4-layer board used, pretty similar to evaluation board by ScioSense.

Connections:

  MSP-port-connector_side-TDC:
 * GPIO in  4.7 right INTERRUPT
 * UCB0CLK  1.4 left  SCK
 * UCB0MISO 1.7 right MISO
 * UCB0SIMO 1.6 right MOSI
 * UCB0STE  1.5 right SSN
 *
 * UCA1TXD  3.4 terminal UART
 * UCA1RXD  3.5
 *
 * REDLED   1.0
 * GRNLED   9.7
 * BTN1     1.1
 * BTN2     1.2
 
 Example terminal otput:
 
 rx_data_result:
0xFF	0xFF	0xFF		0xFF	0xFF	0xFF	/// RAW registers values

0xFF	0xFF	0xFF		0xFF	0xFF	0xFF	


0x43	0x68	0xC5		0x02	0x97	0x05	

0x43	0x68	0xC5		0x02	0x8F	0xAC	

CH1:	0xFFFFFF	16777215	0xFFFFFF		16777215 ps	  ////  channels refid in hex&dec, tstop in hex&dec 

CH2:	0xFFFFFF	16777215	0xFFFFFF		16777215 ps	1-0 0 ps                          /// and difference w prewious value

CH3:	0x4368C5	4417733	0x029705		169733 ps	2-1 -16607482 ps

CH4:	0x4368C5	4417733	0x028FAC		167852 ps	3-2 -1881 ps

rx_data_result:

0xFF	0xFF	0xFF		0xFF	0xFF	0xFF	

0xFF	0xFF	0xFF		0xFF	0xFF	0xFF	

0x5B	0xA6	0x9E		0x02	0x15	0x8E	

0xFF	0xFF	0xFF		0xFF	0xFF	0xFF	

CH1:	0xFFFFFF	16777215  0xFFFFFF		16777215 ps	

CH2:	0xFFFFFF	16777215	0xFFFFFF		16777215 ps	1-0 0 ps

CH3:	0x5BA69E	6006430 0x02158E		136590 ps	2-1 -16640625 ps

CH4:	0xFFFFFF	16777215	0xFFFFFF		16777215 ps	3-2 16640625 ps

all events received
