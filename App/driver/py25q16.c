/* Copyright 2025 muzkr
 * https://github.com/muzkr
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 *     Unless required by applicable law or agreed to in writing, software
 *     distributed under the License is distributed on an "AS IS" BASIS,
 *     WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 *     See the License for the specific language governing permissions and
 *     limitations under the License.
 */

#include <string.h>

#include "driver/py25q16.h"
#include "driver/gpio.h"
#include "py32f071_ll_bus.h"
#include "py32f071_ll_system.h"
#include "py32f071_ll_spi.h"
#include "py32f071_ll_dma.h"
#include "driver/system.h"
#include "driver/systick.h"
#include "external/printf/printf.h"
#include "misc.h"

// #define DEBUG

#define SPIx SPI2
#define CHANNEL_RD LL_DMA_CHANNEL_4
#define CHANNEL_WR LL_DMA_CHANNEL_5

#define CS_PIN GPIO_MAKE_PIN(GPIOA, LL_GPIO_PIN_3)

#define SECTOR_SIZE 0x1000
#define PAGE_SIZE 0x100

// Remplacement du cache secteur (4Ko) par un cache page de seulement 256 octets !
static uint32_t PageCacheAddr = 0x1000000;
static uint8_t PageCache[PAGE_SIZE]; 

static uint8_t BlackHole[4] __attribute__((aligned(4)));
static volatile bool TC_Flag;

static inline void CS_Assert()
{
    GPIO_ResetOutputPin(CS_PIN);
}

static inline void CS_Release()
{
    GPIO_SetOutputPin(CS_PIN);
}

static void SPI_Init()
{
    LL_APB1_GRP1_EnableClock(LL_APB1_GRP1_PERIPH_SPI2);
    LL_AHB1_GRP1_EnableClock(LL_AHB1_GRP1_PERIPH_DMA1);
    LL_IOP_GRP1_EnableClock(LL_IOP_GRP1_PERIPH_GPIOA);

    do
    {
        // SCK: PA0
        // MOSI: PA1
        // MISO: PA2

        LL_GPIO_InitTypeDef InitStruct;
        LL_GPIO_StructInit(&InitStruct);
        InitStruct.Mode = LL_GPIO_MODE_ALTERNATE;
        InitStruct.Speed = LL_GPIO_SPEED_FREQ_VERY_HIGH;
        InitStruct.OutputType = LL_GPIO_OUTPUT_PUSHPULL;
        InitStruct.Pull = LL_GPIO_PULL_UP;

        InitStruct.Pin = LL_GPIO_PIN_0;
        InitStruct.Alternate = LL_GPIO_AF8_SPI2;
        LL_GPIO_Init(GPIOA, &InitStruct);

        InitStruct.Pin = LL_GPIO_PIN_1 | LL_GPIO_PIN_2;
        InitStruct.Alternate = LL_GPIO_AF9_SPI2;
        LL_GPIO_Init(GPIOA, &InitStruct);

    } while (0);

    LL_SYSCFG_SetDMARemap(DMA1, CHANNEL_RD, LL_SYSCFG_DMA_MAP_SPI2_RD);
    LL_SYSCFG_SetDMARemap(DMA1, CHANNEL_WR, LL_SYSCFG_DMA_MAP_SPI2_WR);

    NVIC_SetPriority(DMA1_Channel4_5_6_7_IRQn, 1);
    NVIC_EnableIRQ(DMA1_Channel4_5_6_7_IRQn);

    LL_SPI_InitTypeDef InitStruct;
    LL_SPI_StructInit(&InitStruct);
    InitStruct.Mode = LL_SPI_MODE_MASTER;
    InitStruct.TransferDirection = LL_SPI_FULL_DUPLEX;
    InitStruct.ClockPhase = LL_SPI_PHASE_2EDGE;
    InitStruct.ClockPolarity = LL_SPI_POLARITY_HIGH;
    InitStruct.BaudRate = LL_SPI_BAUDRATEPRESCALER_DIV2;
    InitStruct.BitOrder = LL_SPI_MSB_FIRST;
    InitStruct.NSS = LL_SPI_NSS_SOFT;
    InitStruct.CRCCalculation = LL_SPI_CRCCALCULATION_DISABLE;
    LL_SPI_Init(SPIx, &InitStruct);

    LL_SPI_Enable(SPIx);
}

static void SPI_ReadBuf(uint8_t *Buf, uint32_t Size)
{
    LL_SPI_Disable(SPIx);
    LL_DMA_DisableChannel(DMA1, CHANNEL_RD);
    LL_DMA_DisableChannel(DMA1, CHANNEL_WR);

    LL_DMA_ClearFlag_GI4(DMA1);

    LL_DMA_ConfigTransfer(DMA1, CHANNEL_RD,                 //
                          LL_DMA_DIRECTION_PERIPH_TO_MEMORY //
                              | LL_DMA_MODE_NORMAL          //
                              | LL_DMA_PERIPH_NOINCREMENT   //
                              | LL_DMA_MEMORY_INCREMENT     //
                              | LL_DMA_PDATAALIGN_BYTE      //
                              | LL_DMA_MDATAALIGN_BYTE      //
                              | LL_DMA_PRIORITY_MEDIUM      //
    );

    LL_DMA_ConfigTransfer(DMA1, CHANNEL_WR,                 //
                          LL_DMA_DIRECTION_MEMORY_TO_PERIPH //
                              | LL_DMA_MODE_NORMAL          //
                              | LL_DMA_PERIPH_NOINCREMENT   //
                              | LL_DMA_MEMORY_NOINCREMENT   //
                              | LL_DMA_PDATAALIGN_BYTE      //
                              | LL_DMA_MDATAALIGN_BYTE      //
                              | LL_DMA_PRIORITY_MEDIUM      //
    );

    LL_DMA_SetMemoryAddress(DMA1, CHANNEL_RD, (uint32_t)Buf);
    LL_DMA_SetPeriphAddress(DMA1, CHANNEL_RD, LL_SPI_DMA_GetRegAddr(SPIx));
    LL_DMA_SetDataLength(DMA1, CHANNEL_RD, Size);

    LL_DMA_SetMemoryAddress(DMA1, CHANNEL_WR, (uint32_t)BlackHole);
    LL_DMA_SetPeriphAddress(DMA1, CHANNEL_WR, LL_SPI_DMA_GetRegAddr(SPIx));
    LL_DMA_SetDataLength(DMA1, CHANNEL_WR, Size);

    TC_Flag = false;
    LL_DMA_EnableIT_TC(DMA1, CHANNEL_RD);
    LL_DMA_EnableChannel(DMA1, CHANNEL_RD);
    LL_DMA_EnableChannel(DMA1, CHANNEL_WR);

    LL_SPI_EnableDMAReq_RX(SPIx);
    LL_SPI_Enable(SPIx);
    LL_SPI_EnableDMAReq_TX(SPIx);

    while (!TC_Flag)
        ;
}

static void SPI_WriteBuf(const uint8_t *Buf, uint32_t Size)
{
    LL_SPI_Disable(SPIx);
    LL_DMA_DisableChannel(DMA1, CHANNEL_RD);
    LL_DMA_DisableChannel(DMA1, CHANNEL_WR);

    LL_DMA_ClearFlag_GI4(DMA1);

    LL_DMA_ConfigTransfer(DMA1, CHANNEL_RD,                 //
                          LL_DMA_DIRECTION_PERIPH_TO_MEMORY //
                              | LL_DMA_MODE_NORMAL          //
                              | LL_DMA_PERIPH_NOINCREMENT   //
                              | LL_DMA_MEMORY_NOINCREMENT   //
                              | LL_DMA_PDATAALIGN_BYTE      //
                              | LL_DMA_MDATAALIGN_BYTE      //
                              | LL_DMA_PRIORITY_LOW         //
    );

    LL_DMA_ConfigTransfer(DMA1, CHANNEL_WR,                 //
                          LL_DMA_DIRECTION_MEMORY_TO_PERIPH //
                              | LL_DMA_MODE_NORMAL          //
                              | LL_DMA_PERIPH_NOINCREMENT   //
                              | LL_DMA_MEMORY_INCREMENT     //
                              | LL_DMA_PDATAALIGN_BYTE      //
                              | LL_DMA_MDATAALIGN_BYTE      //
                              | LL_DMA_PRIORITY_LOW         //
    );

    LL_DMA_SetMemoryAddress(DMA1, CHANNEL_RD, (uint32_t)BlackHole);
    LL_DMA_SetPeriphAddress(DMA1, CHANNEL_RD, LL_SPI_DMA_GetRegAddr(SPIx));
    LL_DMA_SetDataLength(DMA1, CHANNEL_RD, Size);

    LL_DMA_SetMemoryAddress(DMA1, CHANNEL_WR, (uint32_t)Buf);
    LL_DMA_SetPeriphAddress(DMA1, CHANNEL_WR, LL_SPI_DMA_GetRegAddr(SPIx));
    LL_DMA_SetDataLength(DMA1, CHANNEL_WR, Size);

    TC_Flag = false;
    LL_DMA_EnableIT_TC(DMA1, CHANNEL_RD);
    LL_DMA_EnableChannel(DMA1, CHANNEL_RD);
    LL_DMA_EnableChannel(DMA1, CHANNEL_WR);

    LL_SPI_EnableDMAReq_RX(SPIx);
    LL_SPI_Enable(SPIx);
    LL_SPI_EnableDMAReq_TX(SPIx);

    while (!TC_Flag)
        ;
}

static uint8_t SPI_WriteByte(uint8_t Value)
{
    while (!LL_SPI_IsActiveFlag_TXE(SPIx))
        ;
    LL_SPI_TransmitData8(SPIx, Value);
    while (!LL_SPI_IsActiveFlag_RXNE(SPIx))
        ;
    return LL_SPI_ReceiveData8(SPIx);
}

static void WriteAddr(uint32_t Addr);
static uint8_t ReadStatusReg(uint32_t Which);
static void WaitWIP();
static void WriteEnable();
static void SectorErase(uint32_t Addr);
static void SectorProgram(uint32_t Addr, const uint8_t *Buf, uint32_t Size);
static void PageProgram(uint32_t Addr, const uint8_t *Buf, uint32_t Size);

void PY25Q16_Init()
{
    CS_Release();
    SPI_Init();
}

void PY25Q16_ReadBuffer(uint32_t Address, void *pBuffer, uint32_t Size)
{
    CS_Assert();

    SPI_WriteByte(0x03);      // Send read command
    WriteAddr(Address);        // Send address (3 bytes)

    // CRITICAL: Flush RX FIFO before DMA to remove residual data
    while (LL_SPI_RX_FIFO_EMPTY != LL_SPI_GetRxFIFOLevel(SPIx))
    {
        LL_SPI_ReceiveData8(SPIx);  // Read and discard
    }

    if (Size >= 16) {
        SPI_ReadBuf((uint8_t *)pBuffer, Size);
    } else {
        for (uint32_t i = 0; i < Size; i++)
        {
            ((uint8_t *)(pBuffer))[i] = SPI_WriteByte(0xff);
        }
    }

    CS_Release();
}

void PY25Q16_WriteBuffer(uint32_t Address, const void *pBuffer, uint32_t Size, bool Append)
{
#ifdef DEBUG
    printf("spi flash write: %06x %ld %d\n", Address, Size, Append);
#endif

    uint32_t PageIndex = Address / PAGE_SIZE;
    uint32_t PageAddr = PageIndex * PAGE_SIZE;
    uint32_t PageOffset = Address % PAGE_SIZE;
    uint32_t PageSize = PAGE_SIZE - PageOffset;

    while (Size)
    {
        WaitWIP();

        if (Size < PageSize)
        {
            PageSize = Size;
        }

        // 1. Lire la page actuelle de 256 octets dans notre petit cache
        if (PageAddr != PageCacheAddr)
        {
            PY25Q16_ReadBuffer(PageAddr, PageCache, PAGE_SIZE);
            PageCacheAddr = PageAddr;
        }

        // 2. Vérifier si les données à écrire sont différentes de ce qui existe déjà
        if (0 != memcmp(pBuffer, PageCache + PageOffset, PageSize))
        {
            bool EraseNeeded = false;
            // Est-ce qu'on essaie d'écrire un '1' sur un '0' ? (Nécessite un effacement)
            for (uint32_t i = 0; i < PageSize; i++)
            {
                uint8_t currentByte = PageCache[PageOffset + i];
                uint8_t newByte = ((const uint8_t *)pBuffer)[i];
                if ((currentByte & newByte) != newByte)
                {
                    EraseNeeded = true;
                    break;
                }
            }

            if (EraseNeeded)
            {
                // Trouver le début du secteur de 4 Ko qui contient cette page
                uint32_t SecAddr = (PageAddr / SECTOR_SIZE) * SECTOR_SIZE;
                
                // Sauvegarder temporairement la page modifiée dans notre cache de 256 octets
                memcpy(PageCache + PageOffset, pBuffer, PageSize);

                // Effacer le secteur entier sur la puce
                SectorErase(SecAddr);
                WaitWIP();

                // Boucle "Read-Modify-Write" intelligente page par page pour reconstruire le secteur
                for (uint32_t p = 0; p < SECTOR_SIZE; p += PAGE_SIZE)
                {
                    uint32_t currentTargetPage = SecAddr + p;

                    if (currentTargetPage == PageAddr)
                    {
                        // C'est la page qu'on vient de modifier en RAM, on l'écrit directement
                        PageProgram(currentTargetPage, PageCache, PAGE_SIZE);
                    }
                    else
                    {
                        // Pour les autres pages du secteur, on les lit et on les réécrit à la volée
                        // (On utilise PageCache comme tampon temporaire de transit)
                        PY25Q16_ReadBuffer(currentTargetPage, PageCache, PAGE_SIZE);
                        
                        // Si la page n'est pas vide, on la réécrit
                        bool pageIsEmpty = true;
                        for(int k=0; k<PAGE_SIZE; k++) {
                            if(PageCache[k] != 0xFF) { pageIsEmpty = false; break; }
                        }
                        if (!pageIsEmpty) {
                            PageProgram(currentTargetPage, PageCache, PAGE_SIZE);
                        }
                    }
                }
                // Forcer le rechargement au prochain tour car le cache a servi au transit
                PageCacheAddr = 0x1000000; 
            }
            else
            {
                // Pas besoin d'effacer ! On applique la modification en RAM et on écrit la page
                memcpy(PageCache + PageOffset, pBuffer, PageSize);
                PageProgram(PageAddr, PageCache, PAGE_SIZE);
            }
        }

        Address += PageSize;
        pBuffer = (const uint8_t *)pBuffer + PageSize;
        Size -= PageSize;

        PageAddr += PAGE_SIZE;
        PageOffset = 0;
        PageSize = PAGE_SIZE;
    }

    WaitWIP();
}

void PY25Q16_SectorErase(uint32_t Address)
{
    Address -= (Address % SECTOR_SIZE);
    SectorErase(Address);
    
    // Si la page en cache était dans ce secteur, on l'invalide
    if (PageCacheAddr >= Address && PageCacheAddr < (Address + SECTOR_SIZE))
    {
        memset(PageCache, 0xff, PAGE_SIZE);
    }
}

// Снять защиту Block Protect перед полным стиранием.
// Читает Status Register 1, сбрасывает биты BP0/BP1/BP2/SRWD и записывает обратно.
// Без этого SectorErase молча игнорируется если защита включена закрытой прошивкой.
void PY25Q16_ClearBlockProtect(void)
{
    // Читаем текущий Status Register 1
    uint8_t sr1 = ReadStatusReg(0);

    // Если биты защиты уже сброшены — ничего не делаем
    // BP0=бит2, BP1=бит3, BP2=бит4, SRWD=бит7
    if ((sr1 & 0x9C) == 0)
        return;

    // Сбрасываем биты защиты: BP0, BP1, BP2, SRWD → 0
    uint8_t new_sr1 = sr1 & ~0x9C;

    WriteEnable();
    WaitWIP();

    CS_Assert();
    SPI_WriteByte(0x01);      // Write Status Register command
    SPI_WriteByte(new_sr1);   // SR1 с обнулёнными битами защиты
    CS_Release();

    WaitWIP();
}

static inline void WriteAddr(uint32_t Addr)
{
    SPI_WriteByte(0xff & (Addr >> 16));
    SPI_WriteByte(0xff & (Addr >> 8));
    SPI_WriteByte(0xff & Addr);
}

static uint8_t ReadStatusReg(uint32_t Which)
{
    uint8_t Cmd;
    switch (Which)
    {
    case 0:
        Cmd = 0x5;
        break;
    case 1:
        Cmd = 0x35;
        break;
    case 2:
        Cmd = 0x15;
        break;
    default:
        return 0;
    }

    CS_Assert();
    SPI_WriteByte(Cmd);
    uint8_t Value = SPI_WriteByte(0xff);
    CS_Release();

    return Value;
}

static void WaitWIP()
{
    for (int i = 0; i < 1000000; i++)
    {
        uint8_t Status = ReadStatusReg(0);
        if (1 & Status) // WIP
        {
            SYSTICK_DelayUs(10);
            continue;
        }
        break;
    }
}

static void WriteEnable()
{
    CS_Assert();
    SPI_WriteByte(0x6);
    CS_Release();
}

static void SectorErase(uint32_t Addr)
{
#ifdef DEBUG
    printf("spi flash sector erase: %06x\n", Addr);
#endif
    WriteEnable();
    WaitWIP();

    CS_Assert();
    SPI_WriteByte(0x20);
    WriteAddr(Addr);
    CS_Release();

    WaitWIP();
}

static void SectorProgram(uint32_t Addr, const uint8_t *Buf, uint32_t Size)
{
    uint32_t Size1 = PAGE_SIZE - (Addr % PAGE_SIZE);

    while (Size)
    {
        if (Size < Size1)
        {
            Size1 = Size;
        }

        PageProgram(Addr, Buf, Size1);

        Addr += Size1;
        Buf += Size1;
        Size -= Size1;

        Size1 = PAGE_SIZE;
    }
}

static void PageProgram(uint32_t Addr, const uint8_t *Buf, uint32_t Size)
{
#ifdef DEBUG
    printf("spi flash page program: %06x %ld\n", Addr, Size);
#endif

    WriteEnable();
    // WaitWIP();

    CS_Assert();

    SPI_WriteByte(0x2);
    WriteAddr(Addr);

    if (Size >= 16)
    {
        SPI_WriteBuf(Buf, Size);
    }
    else
    {
        for (uint32_t i = 0; i < Size; i++)
        {
            SPI_WriteByte(Buf[i]);
        }
    }

    CS_Release();

    WaitWIP();
}

void DMA1_Channel4_5_6_7_IRQHandler()
{
    if (LL_DMA_IsActiveFlag_TC4(DMA1) && LL_DMA_IsEnabledIT_TC(DMA1, CHANNEL_RD))
    {
        LL_DMA_DisableIT_TC(DMA1, CHANNEL_RD);
        LL_DMA_ClearFlag_TC4(DMA1);

        // Wait a tiny bit for SPI to finish
        SYSTICK_DelayUs(10);  // ← ADD THIS

        uint32_t timeout = 10000;
        
        while ((LL_SPI_TX_FIFO_EMPTY != LL_SPI_GetTxFIFOLevel(SPIx)) && timeout--)
            ;
        
        timeout = 10000;
        while (LL_SPI_IsActiveFlag_BSY(SPIx) && timeout--)
            ;
        
        timeout = 10000;
        while ((LL_SPI_RX_FIFO_EMPTY != LL_SPI_GetRxFIFOLevel(SPIx)) && timeout--)
            ;

        LL_SPI_DisableDMAReq_TX(SPIx);
        LL_SPI_DisableDMAReq_RX(SPIx);

        TC_Flag = true;
    }
}