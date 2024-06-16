#include <stdio.h>
#include "boot.h"
#include "main.h"
#include <string.h>
#include <stdbool.h>

#include "flash.h"

extern UART_HandleTypeDef huart3;
#define BL_UART huart3
extern CRC_HandleTypeDef hcrc;

/* Buffer to hold the received data */
static uint8_t Rx_Buffer[ OTA_PACKET_MAX_SIZE ];
/* OTA State */
static OTA_STATE_ ota_state = OTA_STATE_IDLE;

static uint8_t fw_type;
static uint16_t fw_version;
/* Firmware Total Size that we are going to receive */
static uint32_t ota_fw_total_size;
/* Firmware image's CRC32 */
static uint32_t ota_fw_crc;
/* Firmware Size that we have received */
static uint32_t ota_fw_received_size;
/* Slot number to write the received firmware */
static uint8_t slot_num_to_write;
/* Configuration */
OTA_GNRL_CFG_ *cfg_flash   = (OTA_GNRL_CFG_*) (OTA_CONFIG_FLASH_START_ADDR);

/* Hardware CRC handle */
static uint16_t ota_receive_chunk( uint8_t *buf, uint16_t max_len );
static OTA_EX_ ota_process_data( uint8_t *buf, uint16_t len );
static void ota_send_resp( uint8_t cmd , uint8_t type );
static HAL_StatusTypeDef write_data_to_slot( uint8_t slot_num,
                                             uint8_t *data,
                                             uint16_t data_len,
                                             bool is_first_block );
//static HAL_StatusTypeDef write_data_to_flash_app( uint8_t *data, uint32_t data_len );
static uint8_t get_available_slot_number( void );
static HAL_StatusTypeDef write_cfg_to_flash( OTA_GNRL_CFG_ *cfg );



// CRC-16 Lookup Table
static const uint16_t crc16_table[256] = {
    0x0000, 0x1021, 0x2042, 0x3063, 0x4084, 0x50A5, 0x60C6, 0x70E7,
    0x8108, 0x9129, 0xA14A, 0xB16B, 0xC18C, 0xD1AD, 0xE1CE, 0xF1EF,
    0x1231, 0x0210, 0x3273, 0x2252, 0x52B5, 0x4294, 0x72F7, 0x62D6,
    0x9339, 0x8318, 0xB37B, 0xA35A, 0xD3BD, 0xC39C, 0xF3FF, 0xE3DE,
    0x2462, 0x3443, 0x0420, 0x1401, 0x64E6, 0x74C7, 0x44A4, 0x5485,
    0xA56A, 0xB54B, 0x8528, 0x9509, 0xE5EE, 0xF5CF, 0xC5AC, 0xD58D,
    0x3653, 0x2672, 0x1611, 0x0630, 0x76D7, 0x66F6, 0x5695, 0x46B4,
    0xB75B, 0xA77A, 0x9719, 0x8738, 0xF7DF, 0xE7FE, 0xD79D, 0xC7BC,
    0x48C4, 0x58E5, 0x6886, 0x78A7, 0x0840, 0x1861, 0x2802, 0x3823,
    0xC9CC, 0xD9ED, 0xE98E, 0xF9AF, 0x8948, 0x9969, 0xA90A, 0xB92B,
    0x5AF5, 0x4AD4, 0x7AB7, 0x6A96, 0x1A71, 0x0A50, 0x3A33, 0x2A12,
    0xDBFD, 0xCBDC, 0xFBBF, 0xEB9E, 0x9B79, 0x8B58, 0xBB3B, 0xAB1A,
    0x6CA6, 0x7C87, 0x4CE4, 0x5CC5, 0x2C22, 0x3C03, 0x0C60, 0x1C41,
    0xEDAE, 0xFD8F, 0xCDEC, 0xDDCD, 0xAD2A, 0xBD0B, 0x8D68, 0x9D49,
    0x7E97, 0x6EB6, 0x5ED5, 0x4EF4, 0x3E13, 0x2E32, 0x1E51, 0x0E70,
    0xFF9F, 0xEFBE, 0xDFDD, 0xCFFC, 0xBF1B, 0xAF3A, 0x9F59, 0x8F78,
    0x9188, 0x81A9, 0xB1CA, 0xA1EB, 0xD10C, 0xC12D, 0xF14E, 0xE16F,
    0x1080, 0x00A1, 0x30C2, 0x20E3, 0x5004, 0x4025, 0x7046, 0x6067,
    0x83B9, 0x9398, 0xA3FB, 0xB3DA, 0xC33D, 0xD31C, 0xE37F, 0xF35E,
    0x02B1, 0x1290, 0x22F3, 0x32D2, 0x4235, 0x5214, 0x6277, 0x7256,
    0xB5EA, 0xA5CB, 0x95A8, 0x8589, 0xF56E, 0xE54F, 0xD52C, 0xC50D,
    0x34E2, 0x24C3, 0x14A0, 0x0481, 0x7466, 0x6447, 0x5424, 0x4405,
    0xA7DB, 0xB7FA, 0x8799, 0x97B8, 0xE75F, 0xF77E, 0xC71D, 0xD73C,
    0x26D3, 0x36F2, 0x0691, 0x16B0, 0x6657, 0x7676, 0x4615, 0x5634,
    0xD94C, 0xC96D, 0xF90E, 0xE92F, 0x99C8, 0x89E9, 0xB98A, 0xA9AB,
    0x5844, 0x4865, 0x7806, 0x6827, 0x18C0, 0x08E1, 0x3882, 0x28A3,
    0xCB7D, 0xDB5C, 0xEB3F, 0xFB1E, 0x8BF9, 0x9BD8, 0xABBB, 0xBB9A,
    0x4A75, 0x5A54, 0x6A37, 0x7A16, 0x0AF1, 0x1AD0, 0x2AB3, 0x3A92,
    0xFD2E, 0xED0F, 0xDD6C, 0xCD4D, 0xBDAA, 0xAD8B, 0x9DE8, 0x8DC9,
    0x7C26, 0x6C07, 0x5C64, 0x4C45, 0x3CA2, 0x2C83, 0x1CE0, 0x0CC1,
    0xEF1F, 0xFF3E, 0xCF5D, 0xDF7C, 0xAF9B, 0xBFBA, 0x8FD9, 0x9FF8,
    0x6E17, 0x7E36, 0x4E55, 0x5E74, 0x2E93, 0x3EB2, 0x0ED1, 0x1EF0
};

/*
uint32_t CalcCRC(uint8_t * pData, uint32_t DataLength)
{
    uint32_t Checksum = 0xFFFFFFFF;
    for(unsigned int i=0; i < DataLength; i++)
    {
        uint8_t top = (uint8_t)(Checksum >> 24);
        top ^= pData[i];
        Checksum = (Checksum << 8) ^ crc_table[top];
    }
    return Checksum;
}
*/

// Calculate CRC-16
uint16_t CalcCRC(const uint8_t *data, uint32_t length) {
    uint16_t crc = 0xFFFF;
    uint8_t dt=0;
    for (uint32_t i = 0; i < length; i++) {
        crc = (crc << 8) ^ crc16_table[((crc >> 8) ^ data[i]) & 0xFF];
        dt = data[i];
    }

    return crc;
}


/**
  * @brief Download the application from UART and flash it.
  * @param None
  * @retval OTA_EX_
  */
OTA_EX_ ota_download_and_flash( void )
{
  OTA_EX_ ret  = OTA_EX_OK;
  uint16_t    len;

  printf("Waiting for the OTA data...\r\n");
  uint8_t rx_cmd;
  /* Reset the variables */
  ota_fw_total_size    = 0u;
  ota_fw_received_size = 0u;
  ota_fw_crc           = 0u;
  ota_state            = OTA_STATE_START;
  slot_num_to_write    = 0xFFu;
  fw_type			= 0x00;
  fw_version		= 0x0;

  do
  {
    //clear the buffer
    memset( Rx_Buffer, 0, OTA_PACKET_MAX_SIZE );

    len = ota_receive_chunk( Rx_Buffer, OTA_PACKET_MAX_SIZE );
    rx_cmd = Rx_Buffer[1];
    if( len != 0u )
    {
      ret = ota_process_data( Rx_Buffer, len );
      HAL_GPIO_TogglePin(LED_GPIO_Port, LED_Pin);
    }
    else
    {
      //didn't received data. break.
      ret = OTA_EX_ERR;
    }

    //Send ACK or NACK
    if( ret != OTA_EX_OK )
    {
      printf("Sending NACK\r\n");
      ota_send_resp(rx_cmd, OTA_NACK );
      break;
    }
    else
    {
      printf("Sending ACK\r\n");
      ota_send_resp(rx_cmd, OTA_ACK );
    }

  }while( ota_state != OTA_STATE_IDLE );

  return ret;
}

/**
  * @brief Process the received data from UART4.
  * @param buf buffer to store the received data
  * @param max_len maximum length to receive
  * @retval OTA_EX_
  */



static OTA_EX_ ota_process_data( uint8_t *buf, uint16_t len )
{
  OTA_EX_ ret = OTA_EX_ERR;

  do
  {
    if( ( buf == NULL ) || ( len == 0u) )
    {
      break;
    }

    //Check we received OTA Abort command
    switch( ota_state )
    {
      case OTA_STATE_IDLE:
      {
        printf("OTA_STATE_IDLE...\r\n");
        ret = OTA_EX_OK;
      }
      break;

      case OTA_STATE_START:
      {
        OTA_COMMAND_ *cmd = (OTA_COMMAND_*)buf;

	    if( cmd->cmd == OTA_CMD_START )
	    {
		  printf("Received OTA START Command\r\n");
		  ota_state = OTA_STATE_HEADER;
		  ret = OTA_EX_OK;
	    }

      }
      break;

      case OTA_STATE_HEADER:
      {
        OTA_HEADER_ *header = (OTA_HEADER_*)buf;
        if( header->cmd == OTA_CMD_HEADER )
		{
		  ota_fw_total_size = header->meta_data.fw_size;
		  fw_type = header->meta_data.fw_type;
		  ota_fw_crc = header->meta_data.fw_crc;
		  fw_version = header->meta_data.version;
		  printf("Received OTA Header. FW Size = %ld\r\n", ota_fw_total_size);

		  //get the slot number
		  //ota_state = OTA_STATE_DATA;
		  //ret = OTA_EX_OK;

		  slot_num_to_write = get_available_slot_number();
		  if( slot_num_to_write != 0xFF )
		  {
			ota_state = OTA_STATE_DATA;
			ret = OTA_EX_OK;
		  }

		}
      }
      break;

      case OTA_STATE_DATA:
      {

        OTA_DATA_     *data     = (OTA_DATA_*)buf;
        uint16_t          data_len = data->data_len;
        HAL_StatusTypeDef ex;

        if( data->cmd == OTA_CMD_FWDATA )
        {
          bool is_first_block = false;
          if( ota_fw_received_size == 0 )
          {
            //This is the first block
            is_first_block = true;

            /* Read the configuration */
            OTA_GNRL_CFG_ cfg;
            memcpy( &cfg, cfg_flash, sizeof(OTA_GNRL_CFG_) );

            /* Before writing the data, reset the available slot */
            cfg.slot_table[slot_num_to_write].is_this_slot_not_valid = 1u;
            /* write back the updated config */
            ret = write_cfg_to_flash( &cfg );
            if( ret != OTA_EX_OK )
            {
              break;
            }
          }

          /* write the chunk to the Flash (App location) */
          ex = write_data_to_slot( slot_num_to_write, buf+4, data_len, is_first_block );

          if( ex == HAL_OK )
          {
            printf("[%ld/%ld]\r\n", ota_fw_received_size/OTA_DATA_MAX_SIZE, ota_fw_total_size/OTA_DATA_MAX_SIZE);
            if( ota_fw_received_size >= ota_fw_total_size )
            {
              //received the full data. So, move to end
              ota_state = OTA_STATE_END;
            }
            ret = OTA_EX_OK;
          }
        }
      }
      break;

      case OTA_STATE_END:
      {

        OTA_COMMAND_ *cmd = (OTA_COMMAND_*)buf;


          if( cmd->cmd == OTA_CMD_END )
          {
            printf("Received OTA END Command\r\n");

            printf("Validating the received Binary...\r\n");

            uint32_t slot_addr;

            if( fw_type == FW_TYPE_APP )
            {
              slot_addr = OTA_NEW_FW_START_ADDR;
            }
            else
            {
              slot_addr = OTA_NEW_BOOTLOADER_START_ADDR;
            }

            //slot_addr = OTA_APP_SLOT0_FLASH_ADDR;
            //Calculate and verify the CRC
            //uint32_t cal_crc = HAL_CRC_Calculate( &hcrc, (uint32_t*)slot_addr, ota_fw_total_size);
            uint16_t cal_crc = CalcCRC((uint32_t*)slot_addr, ota_fw_total_size);
            //uint16_t cal_data_crc = CalcCRC((uint32_t*)OTA_APP_FLASH_ADDR, cfg.slot_table[slot_num].fw_size);
            if( cal_crc != ota_fw_crc )
            {
              printf("ERROR: FW CRC Mismatch\r\n");
              /* Mayank : removing below break statement for testing purpose */
              //break;
            }
            printf("Done!!!\r\n");

            /* Read the configuration */
            OTA_GNRL_CFG_ cfg;
            memcpy( &cfg, cfg_flash, sizeof(OTA_GNRL_CFG_) );

            //update the slot
            cfg.slot_table[slot_num_to_write].fw_crc                 = cal_crc;
            cfg.slot_table[slot_num_to_write].fw_size                = ota_fw_total_size;
            cfg.slot_table[slot_num_to_write].is_this_slot_not_valid = 0u;
            cfg.slot_table[slot_num_to_write].should_we_run_this_fw  = 1u;
            cfg.slot_table[slot_num_to_write].fw_version			 = fw_version;
            cfg.slot_table[slot_num_to_write].new_app_fw_available 	 = 1u;

            //reset other slots
            for( uint8_t i = 0; i < OTA_NO_OF_SLOTS; i++ )
            {
              if( slot_num_to_write != i )
              {
                //update the slot as inactive
                cfg.slot_table[i].should_we_run_this_fw = 0u;
              }
            }

            //update the reboot reason
            cfg.reboot_cause = OTA_NORMAL_BOOT;

            /* write back the updated config */
            ret = write_cfg_to_flash( &cfg );
            if( ret == OTA_EX_OK )
            {
              ota_state = OTA_STATE_IDLE;
              ret = OTA_EX_OK;
            }
          }

      }
      break;

      default:
      {
        /* Should not come here */
        ret = OTA_EX_ERR;
      }
      break;
    };
  }while( false );

  return ret;
}


/**
  * @brief Receive a one chunk of data.
  * @param buf buffer to store the received data
  * @param max_len maximum length to receive
  * @retval OTA_EX_
  */

static uint16_t ota_receive_chunk( uint8_t *buf, uint16_t max_len )
{
  int16_t  ret;
  uint16_t index        = 0u;
  uint16_t data_len;
  uint16_t cal_data_crc = 0u;
  uint16_t rec_data_crc = 0u;

  do
  {
	//__HAL_UART_CLEAR_OREFLAG(&BL_UART);

    //receive SOF byte (1byte)
    ret = HAL_UART_Receive( &BL_UART, &buf[index], 1, HAL_MAX_DELAY );
    if( ret != HAL_OK )
    {
      break;
    }

	//__HAL_UART_CLEAR_FLAG(&BL_UART,HAL_UART_STATE_BUSY_TX_RX);


    if( buf[index++] != OTA_SOF )
    {
      //Not received start of frame
      ret = OTA_EX_ERR;
      break;
    }

    //Receive the packet type (1byte).
    ret = HAL_UART_Receive( &BL_UART, &buf[index++], 1, PACKET_CAPTURE_TIMEOUT );
    if( ret != HAL_OK )
    {
      break;
    }

    //Get the data length (2bytes).
    ret = HAL_UART_Receive( &BL_UART, &buf[index], 2, PACKET_CAPTURE_TIMEOUT );
    if( ret != HAL_OK )
    {
      break;
    }
    data_len = *(uint16_t *)&buf[index];
    //data_len = buf[index] << 8 | buf[index+1];
    index += 2u;

    for( uint16_t i = 0u; i < data_len; i++ )
    {
      ret = HAL_UART_Receive( &BL_UART, &buf[index++], 1, PACKET_CAPTURE_TIMEOUT );
      if( ret != HAL_OK )
      {
        break;
      }
    }

    if( ret != HAL_OK )
    {
      break;
    }

    //Get the CRC.
    ret = HAL_UART_Receive( &BL_UART, &buf[index], 2, PACKET_CAPTURE_TIMEOUT );
    if( ret != HAL_OK )
    {
      break;
    }
    rec_data_crc = *(uint16_t *)&buf[index];
    //index += 4u;
    //rec_data_crc = buf[index] << 8 | buf[index+1];

    index += 2u;

    //receive EOF byte (1byte)
    ret = HAL_UART_Receive( &BL_UART, &buf[index], 1, PACKET_CAPTURE_TIMEOUT);
    if( ret != HAL_OK )
    {
      break;
    }

    if( buf[index++] != OTA_EOF )
    {
      //Not received end of frame
      ret = OTA_EX_ERR;
      break;
    }

    //Calculate the received data's CRC
    //cal_data_crc = HAL_CRC_Calculate( &hcrc, (uint32_t*)&buf[4], data_len);
    //cal_data_crc = CalcCRC((uint32_t*)&buf[4], data_len);
    // data len + cmd + data
    cal_data_crc = CalcCRC((uint32_t*)&buf[1], data_len+3);

    //Verify the CRC

    if( cal_data_crc != rec_data_crc )
    {
      printf("Chunk's CRC mismatch [Cal CRC = 0x%08lX] [Rec CRC = 0x%08lX]\r\n",
                                                   cal_data_crc, rec_data_crc );
      ret = OTA_EX_ERR;
      break;
    }


  }while( false );

  if( ret != HAL_OK )
  {
    //clear the index if error
    index = 0u;
  }

  if( max_len < index )
  {
    printf("Received more data than expected. Expected = %d, Received = %d\r\n",
                                                              max_len, index );
    index = 0u;
  }

  return index;
}

/*
static uint16_t ota_receive_chunk( uint8_t *buf, uint16_t max_len )
{
  int16_t  ret;
  uint16_t index        = 0u;
  uint16_t data_len;
  uint32_t cal_data_crc = 0u;
  uint32_t rec_data_crc = 0u;

  do
  {
	//__HAL_UART_CLEAR_OREFLAG(&BL_UART);

    //receive SOF byte (1byte)
    ret = HAL_UART_Receive( &BL_UART, &buf[index], 1, HAL_MAX_DELAY );
    if( ret != HAL_OK )
    {
      break;
    }

	//__HAL_UART_CLEAR_FLAG(&BL_UART,HAL_UART_STATE_BUSY_TX_RX);


    if( buf[index++] != OTA_SOF )
    {
      //Not received start of frame
      ret = OTA_EX_ERR;
      break;
    }

    //Receive the packet type (1byte).
    ret = HAL_UART_Receive( &BL_UART, &buf[index++], 1, HAL_MAX_DELAY );
    if( ret != HAL_OK )
    {
      break;
    }

    //Get the data length (2bytes).
    ret = HAL_UART_Receive( &BL_UART, &buf[index], 2, HAL_MAX_DELAY );
    if( ret != HAL_OK )
    {
      break;
    }
    data_len = *(uint16_t *)&buf[index];
    index += 2u;

    for( uint16_t i = 0u; i < data_len; i++ )
    {
      ret = HAL_UART_Receive( &BL_UART, &buf[index++], 1, HAL_MAX_DELAY );
      if( ret != HAL_OK )
      {
        break;
      }
    }

    if( ret != HAL_OK )
    {
      break;
    }

    //Get the CRC.
    ret = HAL_UART_Receive( &BL_UART, &buf[index], 4, HAL_MAX_DELAY );
    if( ret != HAL_OK )
    {
      break;
    }
    rec_data_crc = *(uint32_t *)&buf[index];
    index += 4u;

    //receive EOF byte (1byte)
    ret = HAL_UART_Receive( &BL_UART, &buf[index], 1, HAL_MAX_DELAY );
    if( ret != HAL_OK )
    {
      break;
    }

    if( buf[index++] != OTA_EOF )
    {
      //Not received end of frame
      ret = OTA_EX_ERR;
      break;
    }

    //Calculate the received data's CRC
    //cal_data_crc = HAL_CRC_Calculate( &hcrc, (uint32_t*)&buf[4], data_len);
    cal_data_crc = CalcCRC((uint32_t*)&buf[4], data_len);

    //Verify the CRC

    if( cal_data_crc != rec_data_crc )
    {
      printf("Chunk's CRC mismatch [Cal CRC = 0x%08lX] [Rec CRC = 0x%08lX]\r\n",
                                                   cal_data_crc, rec_data_crc );
      ret = OTA_EX_ERR;
      break;
    }


  }while( false );

  if( ret != HAL_OK )
  {
    //clear the index if error
    index = 0u;
  }

  if( max_len < index )
  {
    printf("Received more data than expected. Expected = %d, Received = %d\r\n",
                                                              max_len, index );
    index = 0u;
  }

  return index;
}
*/

/**
  * @brief Send the response.
  * @param type ACK or NACK
  * @retval none
  */
static void ota_send_resp( uint8_t cmd , uint8_t type )
{
  OTA_RESP_ rsp =
  {
    .sof         = OTA_SOF,
    .cmd = cmd,
    .data_len    = 1u,
    .status      = type,
    .eof         = OTA_EOF
  };

  rsp.crc = CalcCRC((uint16_t*)&rsp.status, 1);
  //send response
  HAL_UART_Transmit(&BL_UART, (uint8_t *)&rsp, sizeof(OTA_RESP_), HAL_MAX_DELAY);
}

/**
  * @brief Write data to the Slot
  * @param slot_num slot to be written
  * @param data data to be written
  * @param data_len data length
  * @is_first_block true - if this is first block, false - not first block
  * @retval HAL_StatusTypeDef
  */

static HAL_StatusTypeDef write_data_to_slot( uint8_t slot_num,
                                             uint8_t *data,
                                             uint16_t data_len,
                                             bool is_first_block )
{
  HAL_StatusTypeDef ret;
  uint32_t FirstPage = 0, NbOfPages = 0, BankNumber = 0;
  uint32_t Address = 0, PAGEError = 0;
  __IO uint32_t data32 = 0 , MemoryProgramStatus = 0;
  do
  {

    if( slot_num >= OTA_NO_OF_SLOTS )
    {
      ret = HAL_ERROR;
      break;
    }

    ret = HAL_FLASH_Unlock();
    if( ret != HAL_OK )
    {
      break;
    }

    //No need to erase every time. Erase only the first time.
    if( is_first_block )
    {
        printf("Erasing the Slot %d Flash memory...\r\n", slot_num);
        //Erase the Flash
        FLASH_EraseInitTypeDef EraseInitStruct;
		HAL_FLASH_Unlock();

		uint32_t erase_start_addr,erase_end_addr;
		if(fw_type == FW_TYPE_APP)
		{
			erase_start_addr = OTA_NEW_FW_START_ADDR;
			erase_end_addr = OTA_NEW_FW_END_ADDR;
		}
		else
		{
			erase_start_addr = OTA_NEW_BOOTLOADER_START_ADDR;
			erase_end_addr = OTA_NEW_BOOTLOADER_END_ADDR;
		}
		FirstPage = GetPage(erase_start_addr);
		/* Get the number of pages to erase from 1st page */
		NbOfPages = GetPage(erase_end_addr) - FirstPage;
		/* Get the bank */
		BankNumber = GetBank(FLASH_USER_START_ADDR);
		/* Fill EraseInit structure*/
		EraseInitStruct.TypeErase   = FLASH_TYPEERASE_PAGES;
		EraseInitStruct.Banks       = BankNumber;
		EraseInitStruct.Page        = FirstPage;
		EraseInitStruct.NbPages     = NbOfPages;
		ret = HAL_FLASHEx_Erase(&EraseInitStruct, &PAGEError);
		HAL_FLASH_Lock();

	  if( ret != HAL_OK )
	  {
		printf("Flash Erase Error\r\n");
		break;
	  }
    }

    uint32_t flash_addr = fw_type == FW_TYPE_APP ? OTA_NEW_FW_START_ADDR : OTA_NEW_BOOTLOADER_START_ADDR;

	HAL_FLASH_Unlock();

    for(int i = 0; i < data_len; i += 8 )
    {
      uint64_t data64=0;
      for(int j=0; j<8; j++)
      {
    	  data64 |= (uint64_t)data[i + j] << (j * 8);
      }
      ret = HAL_FLASH_Program(FLASH_TYPEPROGRAM_DOUBLEWORD, (flash_addr + ota_fw_received_size), data64);

      if( ret == HAL_OK )
      {
        //update the data count
        ota_fw_received_size += 8;
      }
      else
      {
        printf("Flash Write Error\r\n");
        break;
      }
    }

    if( ret != HAL_OK )
    {
      break;
    }

    ret = HAL_FLASH_Lock();
    if( ret != HAL_OK )
    {
      break;
    }
  }while( false );

  HAL_FLASH_Lock();
  return ret;
}

/**
  * @brief Return the available slot number
  * @param none
  * @retval slot number
  */
static uint8_t get_available_slot_number( void )
{
  uint8_t   slot_number = 0xFF;

  /* Read the configuration */
  OTA_GNRL_CFG_ cfg;
  memcpy( &cfg, cfg_flash, sizeof(OTA_GNRL_CFG_) );
  /*
   * Check the slot is valid or not. If it is valid,
   * then check the slot is active or not.
   *
   * If it is valid and not active, then use that slot.
   * If it is not valid, then use that slot.
   *
   */

   for( uint8_t i = 0; i < OTA_NO_OF_SLOTS; i++ )
   {
     if( ( cfg.slot_table[i].is_this_slot_not_valid != 0u ) || ( cfg.slot_table[i].is_this_slot_active == 0u ) )
     {
       slot_number = i;
       printf("Slot %d is available for OTA update\r\n", slot_number);
       break;
     }
   }

   return 0;
}


/**
  * @brief Write data to the Application's actual flash location.
  * @param data data to be written
  * @param data_len data length
  * @retval HAL_StatusTypeDef
  */
//static HAL_StatusTypeDef write_data_to_flash_app( uint8_t *data, uint32_t data_len )
//{
//  HAL_StatusTypeDef ret;
//  uint32_t FirstPage = 0, NbOfPages = 0, BankNumber = 0;
//  uint32_t PAGEError = 0;
//  static FLASH_EraseInitTypeDef EraseInitStruct;
//
//  do
//  {
//    ret = HAL_FLASH_Unlock();
//    if( ret != HAL_OK )
//    {
//      break;
//    }
//    //Check if the FLASH_FLAG_BSY.
//    FLASH_WaitForLastOperation( HAL_MAX_DELAY );
//    printf("Erasing the App Flash memory...\r\n");
//    //Erase the Flash
//	FirstPage = GetPage(OTA_APP_FLASH_ADDR);
//	/* Get the number of pages to erase from 1st page */
//	NbOfPages = GetPage(ADDR_FLASH_PAGE_127) - FirstPage + 1;
//	/* Get the bank */
//	BankNumber = GetBank(FLASH_USER_START_ADDR);
//	/* Fill EraseInit structure*/
//	EraseInitStruct.TypeErase   = FLASH_TYPEERASE_PAGES;
//	EraseInitStruct.Banks       = BankNumber;
//	EraseInitStruct.Page        = FirstPage;
//	EraseInitStruct.NbPages     = NbOfPages;
//	ret = HAL_FLASHEx_Erase(&EraseInitStruct, &PAGEError);
//
//    if( ret != HAL_OK )
//    {
//      printf("Flash erase Error\r\n");
//      break;
//    }
//
//    for( uint32_t i = 0; i < data_len; i += 8 )
//    {
//
//	 uint64_t data64=0;
//
//	 for(int j=0; j<8; j++)
//	 {
//		 data64 |= (uint64_t)data[i + j] << (j * 8);
//	 }
//
//      ret = HAL_FLASH_Program(FLASH_TYPEPROGRAM_DOUBLEWORD, OTA_APP_FLASH_ADDR + i, data64);
//      if( ret != HAL_OK )
//      {
//        printf("App Flash Write Error\r\n");
//        break;
//      }
//    }
//
//    if( ret != HAL_OK )	break;
//    ret = HAL_FLASH_Lock();
//    if( ret != HAL_OK )	break;
//
//
//    //Check if the FLASH_FLAG_BSY.
//    FLASH_WaitForLastOperation( HAL_MAX_DELAY );
//
//  }while( false );
//
//  return ret;
//}

/**
  * @brief Load the new app to the app's actual flash memory.
  * @param none
  * @retval none
  */
//void load_new_app( void )
//{
//  bool              is_update_available = false;
//  uint8_t           slot_num;
//  HAL_StatusTypeDef ret;
//
//  /* Read the configuration */
//  OTA_GNRL_CFG_ cfg;
//  memcpy( &cfg, cfg_flash, sizeof(OTA_GNRL_CFG_) );
//
//  /*
//   * Check the slot whether it has a new application.
//   */
//
//   for( uint8_t i = 0; i < OTA_NO_OF_SLOTS; i++ )
//   {
//     if( cfg.slot_table[i].should_we_run_this_fw == 1u )
//     {
//       printf("New Application is available in the slot %d!!!\r\n", i);
//       is_update_available               = true;
//       slot_num                          = i;
//
//       //update the slot
//       cfg.slot_table[i].is_this_slot_active    = 1u;
//       cfg.slot_table[i].should_we_run_this_fw  = 0u;
//
//       break;
//     }
//   }
//
//   if( is_update_available )
//   {
//     //make other slots inactive
//     for( uint8_t i = 0; i < OTA_NO_OF_SLOTS; i++ )
//     {
//       if( slot_num != i )
//       {
//         //update the slot as inactive
//         cfg.slot_table[i].is_this_slot_active = 0u;
//       }
//     }
//
//
//
//     uint32_t slot_addr;
//     if( slot_num == 0u )
//     {
//       slot_addr = OTA_APP_SLOT0_FLASH_ADDR;
//     }
//     else
//     {
//       slot_addr = OTA_APP_SLOT1_FLASH_ADDR;
//     }
//
//     //Load the new app or firmware to app's flash address
//     ret = write_data_to_flash_app( (uint8_t*)slot_addr, cfg.slot_table[slot_num].fw_size );
//     if( ret != HAL_OK )
//     {
//       printf("App Flash write Error\r\n");
//     }
//     else
//     {
//       // write back the updated config
//       ret = write_cfg_to_flash( &cfg );
//       if( ret != HAL_OK )
//       {
//         printf("Config Flash write Error\r\n");
//       }
//     }
//   }
//   else
//   {
//     //Find the active slot in case the update is not available
//     for( uint8_t i = 0; i < OTA_NO_OF_SLOTS; i++ )
//     {
//       if( cfg.slot_table[i].is_this_slot_active == 1u )
//       {
//         slot_num = i;
//         break;
//       }
//     }
//   }
//  uint16_t slot_addr;
//  if( slot_num == 0u )
//  {
//    slot_addr = OTA_APP_SLOT0_FLASH_ADDR;
//  }
//  else
//  {
//    slot_addr = OTA_APP_SLOT1_FLASH_ADDR;
//  }
//   //Verify the application is corrupted or not
//   printf("Verifying the Application...");
//
//   FLASH_WaitForLastOperation( HAL_MAX_DELAY );
//   //Verify the application
//   //uint32_t cal_data_crc = HAL_CRC_Calculate( &hcrc, (uint32_t*)OTA_APP_FLASH_ADDR, cfg.slot_table[slot_num].fw_size );
//   uint16_t cal_data_crc = CalcCRC((uint32_t*)OTA_APP_FLASH_ADDR, cfg.slot_table[slot_num].fw_size);
//   FLASH_WaitForLastOperation( HAL_MAX_DELAY );
//   //Verify the CRC
//   if( cal_data_crc != cfg.slot_table[slot_num].fw_crc )
//   {
//     printf("ERROR!!!\r\n");
//     printf("Invalid Application. HALT!!!\r\n");
//
//
//     while(1);
//   }
//   printf("Done!!!\r\n");
//}


/**
  * @brief Write the configuration to flash
  * @param cfg config structure
  * @retval none
  */
static HAL_StatusTypeDef write_cfg_to_flash( OTA_GNRL_CFG_ *cfg )
{
  HAL_StatusTypeDef ret;
  uint32_t FirstPage = 0, NbOfPages = 0, BankNumber = 0;
  uint32_t PAGEError = 0;
  static FLASH_EraseInitTypeDef EraseInitStruct;

  do
  {
    if( cfg == NULL )
    {
      ret = HAL_ERROR;
      break;
    }

    ret = HAL_FLASH_Unlock();
    if( ret != HAL_OK )	break;

    //Check if the FLASH_FLAG_BSY.
    FLASH_WaitForLastOperation( HAL_MAX_DELAY );
	HAL_FLASH_Unlock();
	FirstPage = GetPage(OTA_CONFIG_FLASH_START_ADDR);
	/* Get the number of pages to erase from 1st page */
	NbOfPages = GetPage(OTA_CONFIG_FLASH_END_ADDR) - FirstPage;
	/* Get the bank */
	BankNumber = GetBank(FLASH_USER_START_ADDR);
	/* Fill EraseInit structure*/
	EraseInitStruct.TypeErase   = FLASH_TYPEERASE_PAGES;
	EraseInitStruct.Banks       = BankNumber;
	EraseInitStruct.Page        = FirstPage;
	EraseInitStruct.NbPages     = NbOfPages;
	ret = HAL_FLASHEx_Erase(&EraseInitStruct, &PAGEError);
    if( ret != HAL_OK )	break;
    //write the configuration
    uint8_t *data = (uint8_t *) cfg;
    for( uint32_t i = 0u; i < sizeof(OTA_GNRL_CFG_); i+=8 )
    {
	  uint64_t data64=0;
	  for(int j=0; j<8; j++)
	  {
		 data64 |= (uint64_t)data[i + j] << (j * 8);
	  }

      ret = HAL_FLASH_Program( FLASH_TYPEPROGRAM_DOUBLEWORD,
                               OTA_CONFIG_FLASH_START_ADDR + i,
							   data64);
      if( ret != HAL_OK )
      {
        printf("Slot table Flash Write Error\r\n");
        break;
      }

    }

    //Check if the FLASH_FLAG_BSY.
    FLASH_WaitForLastOperation( HAL_MAX_DELAY );
    //ret = HAL_OK;
    if( ret != HAL_OK )	break;

    ret = HAL_FLASH_Lock();
    if( ret != HAL_OK )	break;

  }
  while( false );

  return ret;
}
