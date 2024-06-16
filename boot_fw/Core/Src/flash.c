
#include "flash.h"
#include "main.h"

/* Clear flags */
void FLASH_If_Init(void)
{
	HAL_FLASH_Unlock();
	__HAL_FLASH_CLEAR_FLAG(FLASH_FLAG_OPTVERR);
	HAL_FLASH_Lock();
}

uint32_t GetPage(uint32_t Addr)
{
  uint32_t page = 0;

  if (Addr < (FLASH_BASE + FLASH_BANK_SIZE))
  {
    /* Bank 1 */
    page = (Addr - FLASH_BASE) / FLASH_PAGE_SIZE;
  }
  else
  {
    /* Bank 2 */
    page = (Addr - (FLASH_BASE + FLASH_BANK_SIZE)) / FLASH_PAGE_SIZE;
  }

  return page;
}

/**
  * @brief  Gets the bank of a given address
  * @param  Addr: Address of the FLASH Memory
  * @retval The bank of a given address
  */
uint32_t GetBank(uint32_t Addr)
{
  return FLASH_BANK_1;
}


/* Erase flash memory */
uint32_t FLASH_If_Erase(uint32_t start)
{
	uint32_t FirstPage = 0, NbOfPages = 0, BankNumber = 0;
	uint32_t Address = 0, PAGEError = 0;
	__IO uint32_t data32 = 0 , MemoryProgramStatus = 0;
	static FLASH_EraseInitTypeDef EraseInitStruct;
	HAL_StatusTypeDef status = HAL_OK;
	HAL_FLASH_Unlock();
	FirstPage = GetPage(start);
	/* Get the number of pages to erase from 1st page */
	NbOfPages = GetPage(FLASH_USER_END_ADDR) - FirstPage + 1;
	/* Get the bank */
	BankNumber = GetBank(FLASH_USER_START_ADDR);
	/* Fill EraseInit structure*/
	EraseInitStruct.TypeErase   = FLASH_TYPEERASE_PAGES;
	EraseInitStruct.Banks       = BankNumber;
	EraseInitStruct.Page        = FirstPage;
	EraseInitStruct.NbPages     = NbOfPages;
	status = HAL_FLASHEx_Erase(&EraseInitStruct, &PAGEError);
	HAL_FLASH_Lock();

	if (status != HAL_OK)
	{
		return FLASHIF_ERASEKO;
	}

	return FLASHIF_OK;
}

/* Write flash memory */
uint32_t FLASH_If_Write(uint32_t destination, uint32_t *p_source, uint32_t length)
{
	uint32_t i = 0;

	HAL_FLASH_Unlock();

	for (i = 0; (i < length) && (destination <= (FLASH_USER_END_ADDR-4)); i++)
	{
		//HAL_StatusTypeDef HAL_FLASH_Program(uint32_t TypeProgram, uint32_t Address, uint64_t Data)
		if (HAL_FLASH_Program(FLASH_TYPEPROGRAM_FAST, destination, *(uint32_t*)(p_source+i)) == HAL_OK)
		//if (HAL_FLASH_Program(FLASH_TYPEPROGRAM_WORD, destination, *(uint32_t*)(p_source+i)) == HAL_OK)
		{
			/* Validate the written value */
			if (*(uint32_t*)destination != *(uint32_t*)(p_source+i))
			{
				HAL_FLASH_Lock();
				return (FLASHIF_WRITINGCTRL_ERROR);
			}

			/* Increase WORD length */
			destination += 4;
		}
		else
		{
			HAL_FLASH_Lock();
			return (FLASHIF_WRITING_ERROR);
		}
	}

	HAL_FLASH_Lock();
	return (FLASHIF_OK);
}
