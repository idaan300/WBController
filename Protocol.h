#pragma once


typedef enum {

	RB_PROT_INIT = 0xAA,
	RB_PROT_UNDEFINED = 0x00,
	RB_PROT_ROBOTDATA = 0x01,
	RB_PROT_FLAGDATA = 0x02,
	RB_PROT_MAPDATA = 0x03,
	RB_PROT_EMGSTOP = 0xFF,

} TCPCommand;