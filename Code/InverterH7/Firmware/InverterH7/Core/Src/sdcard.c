/*
 * sd.c
 *
 *  Created on: Aug 14, 2025
 *      Author: LiongMa
 */


#include "sdcard.h"

extern SD_HandleTypeDef hsd1;
extern UART_HandleTypeDef huart1;

char TxBuffer[250];

void SDIO_SDCard_Test(void)
{
  FATFS FatFs;
  FIL Fil;
  FRESULT FR_Status;
  FATFS *FS_Ptr;
  UINT WWC; // Read/Write Word Counter
  DWORD FreeClusters;
  uint32_t TotalSize, FreeSpace;
  char RW_Buffer[200];
  sprintf(TxBuffer, "test \r\n");
  HAL_UART_Transmit_DMA(&huart1, (uint8_t*)TxBuffer, strlen(TxBuffer));
  do
  {
    //------------------[ Mount The SD Card ]--------------------
    FR_Status = f_mount(&FatFs, SDPath, 1);
    if (FR_Status != FR_OK)
    {
      sprintf(TxBuffer, "Error! While Mounting SD Card, Error Code: (%i)\r\n", FR_Status);
		HAL_UART_Transmit_DMA(&huart1, (uint8_t*)TxBuffer, strlen(TxBuffer));
      break;
    }
    sprintf(TxBuffer, "SD Card Mounted Successfully! \r\n\n");
	HAL_UART_Transmit_DMA(&huart1, (uint8_t*)TxBuffer, strlen(TxBuffer));
    //------------------[ Get & Print The SD Card Size & Free Space ]--------------------
    f_getfree("", &FreeClusters, &FS_Ptr);
    TotalSize = (uint32_t)((FS_Ptr->n_fatent - 2) * FS_Ptr->csize * 0.5);
    FreeSpace = (uint32_t)(FreeClusters * FS_Ptr->csize * 0.5);
    sprintf(TxBuffer, "Total SD Card Size: %lu Bytes\r\n", TotalSize);
	HAL_UART_Transmit_DMA(&huart1, (uint8_t*)TxBuffer, strlen(TxBuffer));
    sprintf(TxBuffer, "Free SD Card Space: %lu Bytes\r\n\n", FreeSpace);
	HAL_UART_Transmit_DMA(&huart1, (uint8_t*)TxBuffer, strlen(TxBuffer));
    //------------------[ Open A Text File For Write & Write Data ]--------------------
    //Open the file
    FR_Status = f_open(&Fil, "MyTextFile.txt", FA_WRITE | FA_READ | FA_CREATE_ALWAYS);
    if(FR_Status != FR_OK)
    {
      sprintf(TxBuffer, "Error! While Creating/Opening A New Text File, Error Code: (%i)\r\n", FR_Status);
		HAL_UART_Transmit_DMA(&huart1, (uint8_t*)TxBuffer, strlen(TxBuffer));
      break;
    }
    sprintf(TxBuffer, "Text File Created & Opened! Writing Data To The Text File..\r\n\n");
	HAL_UART_Transmit_DMA(&huart1, (uint8_t*)TxBuffer, strlen(TxBuffer));
    // (1) Write Data To The Text File [ Using f_puts() Function ]
    f_puts("Hello! From STM32 To SD Card Over SDMMC, Using f_puts()\n", &Fil);
    // (2) Write Data To The Text File [ Using f_write() Function ]
    strcpy(RW_Buffer, "Hello! From STM32 To SD Card Over SDMMC, Using f_write()\r\n");
    f_write(&Fil, RW_Buffer, strlen(RW_Buffer), &WWC);
    // Close The File
    f_close(&Fil);

  } while(0);
  //------------------[ Test Complete! Unmount The SD Card ]--------------------
  FR_Status = f_mount(NULL, "", 0);
  if (FR_Status != FR_OK)
  {
      sprintf(TxBuffer, "\r\nError! While Un-mounting SD Card, Error Code: (%i)\r\n", FR_Status);
		HAL_UART_Transmit_DMA(&huart1, (uint8_t*)TxBuffer, strlen(TxBuffer));
  } else{
      sprintf(TxBuffer, "\r\nSD Card Un-mounted Successfully! \r\n");
		HAL_UART_Transmit_DMA(&huart1, (uint8_t*)TxBuffer, strlen(TxBuffer));
  }
}

