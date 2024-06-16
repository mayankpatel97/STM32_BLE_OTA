#ifndef BOOT_H
#define BOOT_H

#include <stdbool.h>
#include "main.h"
#include "flash.h"

#define OTA_SOF  0x2A    // Start of Frame
#define OTA_EOF  0x23    // End of Frame
#define OTA_ACK  0x00    // ACK
#define OTA_NACK 0x01    // NACK

#define PACKET_CAPTURE_TIMEOUT 250


/* active bootloader : 2KB
 * active application:239KB
 * new application	 :239KB
 * new bootloader	:16KB
 * information : 2KB
 */
/*
#define OTA_APP_FLASH_ADDR        0x08020000   //Application's Flash Address
#define OTA_APP_SLOT0_FLASH_ADDR  0x08040000   //App slot 0 address
#define OTA_APP_SLOT1_FLASH_ADDR  0x08040000   //App slot 1 address
//#define OTA_CONFIG_FLASH_ADDR     0x08020000   //Configuration's address
#define OTA_CONFIG_FLASH_ADDR     0x08060000   //Configuration's address
*/


//#define OTA_APP_FLASH_ADDR           //Application's Flash Address
#define OTA_ACTV_BOOTLOADER_START_ADDR		ADDR_FLASH_PAGE_0
#define OTA_ACTV_BOOTLOADER_END_ADDR		ADDR_FLASH_PAGE_24
#define OTA_ACTV_FW_START_ADDR		ADDR_FLASH_PAGE_25
#define OTA_ACTV_FW_END_ADDR		ADDR_FLASH_PAGE_125

#define OTA_NEW_FW_START_ADDR		ADDR_FLASH_PAGE_126
#define OTA_NEW_FW_END_ADDR			ADDR_FLASH_PAGE_225

#define OTA_NEW_BOOTLOADER_START_ADDR  ADDR_FLASH_PAGE_226
#define OTA_NEW_BOOTLOADER_END_ADDR		ADDR_FLASH_PAGE_251
#define OTA_CONFIG_FLASH_START_ADDR     ADDR_FLASH_PAGE_252   //Configuration's address
#define OTA_CONFIG_FLASH_END_ADDR		ADDR_FLASH_PAGE_255

#define FW_TYPE_APP			0x01
#define FW_TYPE_BOOTLDR		0x02


#define OTA_NO_OF_SLOTS           1            //Number of slots
#define OTA_SLOT_MAX_SIZE        (128 * 1024)  //Each slot size (512KB)

#define OTA_DATA_MAX_SIZE ( 128 )  //Maximum data Size
#define OTA_DATA_OVERHEAD (    9 )  //data overhead
#define OTA_PACKET_MAX_SIZE ( OTA_DATA_MAX_SIZE + OTA_DATA_OVERHEAD )

/*
 * Reboot reason
 */
#define OTA_FIRST_TIME_BOOT       ( 0xFFFFFFFF )      //First time boot
#define OTA_NORMAL_BOOT           ( 0xBEEFFEED )      //Normal Boot
#define OTA_REQUEST           ( 0xDEADBEEF )      //OTA request by application
#define OTA_LOAD_PREV_APP         ( 0xFACEFADE )      //App requests to load the previous version

/*
 * Exception codes
 */
typedef enum
{
  OTA_EX_OK       = 0,    // Success
  OTA_EX_ERR      = 1,    // Failure
}OTA_EX_;

/*
 * OTA process state
 */
typedef enum
{
  OTA_STATE_IDLE    = 0,
  OTA_STATE_START   = 1,
  OTA_STATE_HEADER  = 2,
  OTA_STATE_DATA    = 3,
  OTA_STATE_END     = 4,
}OTA_STATE_;

/*
 * Packet type
 */
typedef enum
{
  OTA_PACKET_TYPE_CMD       = 0,    // Command
  OTA_PACKET_TYPE_DATA      = 1,    // Data
  OTA_PACKET_TYPE_HEADER    = 2,    // Header
  OTA_PACKET_TYPE_RESPONSE  = 3,    // Response
}OTA_PACKET_TYPE_;

/*
 * OTA Commands
 */
typedef enum
{
  OTA_CMD_START = 1,    // OTA Start command
  OTA_CMD_HEADER = 2,
  OTA_CMD_FWDATA = 3,
  OTA_CMD_END   = 4,    // OTA End command
  OTA_CMD_ABORT = 5,    // OTA Abort command
}OTA_CMD_;

/*
 * Slot table
 */
typedef struct
{
    uint8_t  is_this_slot_not_valid;  //Is this slot has a valid firmware/application?
    uint8_t  is_this_slot_active;     //Is this slot's firmware is currently running?
    uint8_t  should_we_run_this_fw;   //Do we have to run this slot's firmware?
    uint32_t fw_size;                 //Slot's firmware/application size
    uint32_t fw_crc;                  //Slot's firmware/application CRC
    uint16_t fw_version;
    uint8_t new_app_fw_available;
    uint8_t reserved1;
    uint32_t reserved2;
    uint32_t reserved3;
}__attribute__((packed)) OTA_SLOT_;

/*
 * General configuration
 */
typedef struct
{
    uint32_t  reboot_cause;
    OTA_SLOT_ slot_table[OTA_NO_OF_SLOTS];
}__attribute__((packed)) OTA_GNRL_CFG_;

/*
 * OTA meta info
 */
typedef struct
{
  uint32_t fw_size;
  uint8_t fw_type;
  uint16_t fw_crc;
  uint16_t version;

}__attribute__((packed)) meta_info;

/*
 * OTA Command format
 *
 * ________________________________________
 * |     | Packet |     |     |     |     |
 * | SOF | Type   | Len | CMD | CRC | EOF |
 * |_____|________|_____|_____|_____|_____|
 *   1B      1B     2B    1B     4B    1B
 */
typedef struct
{
  uint8_t   sof;
  uint8_t   cmd;
  uint16_t  data_len;
  uint8_t   data;
  uint16_t  crc;
  uint8_t   eof;
}__attribute__((packed)) OTA_COMMAND_;

/*
 * OTA Header format
 *
 * __________________________________________
 * |     | Packet |     | Header |     |     |
 * | SOF | Type   | Len |  Data  | CRC | EOF |
 * |_____|________|_____|________|_____|_____|
 *   1B      1B     2B     16B     4B    1B
 */
typedef struct
{
  uint8_t     sof;
  uint8_t     cmd;
  uint16_t    data_len;
  meta_info   meta_data;
  uint16_t    crc;
  uint8_t     eof;
}__attribute__((packed)) OTA_HEADER_;

/*
 * OTA Data format
 *
 * __________________________________________
 * |     | Packet |     |        |     |     |
 * | SOF | Type   | Len |  Data  | CRC | EOF |
 * |_____|________|_____|________|_____|_____|
 *   1B      1B     2B    nBytes   4B    1B
 */
typedef struct
{
  uint8_t     sof;
  uint8_t     cmd;
  uint16_t    data_len;
  uint8_t     *data;
}__attribute__((packed)) OTA_DATA_;

/*
 * OTA Response format
 *
 * __________________________________________
 * |     | Packet |     |        |     |     |
 * | SOF | Type   | Len | Status | CRC | EOF |
 * |_____|________|_____|________|_____|_____|
 *   1B      1B     2B      1B     4B    1B
 */
typedef struct
{
  uint8_t   sof;
  uint8_t   cmd;
  uint16_t  data_len;
  uint8_t   status;
  uint16_t  crc;
  uint8_t   eof;
}__attribute__((packed)) OTA_RESP_;


void load_new_app( void );
#endif /* BOOT_H */
