/**
 * mcalDMA.h
 *
 *  Created on: 26.10.2020
 *      Author: Ralf Jesse
 *       Email: embedded@ralf-jesse.de
 */

#ifndef MCALDMAC_H_
#define MCALDMAC_H_

#include <stm32f4xx.h>
#include <stdint.h>
#include <stdbool.h>

#ifdef __cplusplus
extern "C" {
#endif


//#include <mcalParamCheck.h>

/**
 * @ingoup dma5
 * @{
 */
#define DMA_MAX_DATA_LEN    (65536)

typedef enum
{
    DMAC_OK                         =   0,
    DMAC_INVALID_DMAC               = -20,
    DMAC_INVALID_STREAM             = -21,
    DMAC_INVALID_CHANNEL            = -22,
    DMAC_INVALID_DIR                = -23,
    DMAC_INVALID_DATA_LEN           = -24,
    DMAC_INVALID_DATA_FORMAT        = -25,
    DMAC_INVALID_SRC_DATA_FORMAT    = -26,
    DMAC_INVALID_DEST_DATA_FORMAT   = -27,
    DMAC_INVALID_IRQ_TYPE           = -28,
    DMAC_INVALID_PRIORITY_SETTING   = -29,
    DMAC_INVALID_INCREMENT          = -30,
    DMAC_INVALID_INCREMENTAL_MODE   = -31,
    DMAC_INVALID_NUM_BEATS          = -32,
    DMAC_INVALID_FLOW_CONTROLLER    = -33,
    DMAC_INVALID_OFFSET_SIZE        = -34,
    DMAC_NOT_IN_DOUBLE_BUFFER_MODE  = -35,
    DMAC_INVALID_TARGET_MEMORY      = -36,
    DMAC_INVALID_THRESHOLD_LEVEL    = -37
} DMAC_RETURN_CODE_t;

typedef enum
{
    // New names
    DMA_CHN_0 = 0,
    DMA_CHN_1 = 1,
    DMA_CHN_2 = 2,
    DMA_CHN_3 = 3,
    DMA_CHN_4 = 4,
    DMA_CHN_5 = 5,
    DMA_CHN_6 = 6,
    DMA_CHN_7 = 7,

    // Old names (kept for compatibility with old version)
    CHANNEL_0 = 0,
    CHANNEL_1 = 1,
    CHANNEL_2 = 2,
    CHANNEL_3 = 3,
    CHANNEL_4 = 4,
    CHANNEL_5 = 5,
    CHANNEL_6 = 6,
    CHANNEL_7 = 7
} DMAC_CHANNEL_t;

typedef enum
{
    PER_2_MEM,          // Peripheral --> Memory
    MEM_2_PER,          // Memory     --> Peripheral
    MEM_2_MEM,          // Memory     --> Memory
} DMAC_DIRECTION_t;

typedef enum
{
    BYTE,
    HALFWORD,
    WORD
} DMAC_DATA_FORMAT_t;

typedef enum
{
    FIFO_OFF,
    FIFO_ON
} DMAC_FIFO_MODE_t;

typedef enum
{
    DIRECT_MODE_ERR   =   2,
    TX_ERR            =   4,
    TX_HALF           =   8,
    TX_COMPLETE       =  16,
    FIFO_ERROR_ENABLE = 128
} DMAC_IRQ_t;

typedef enum
{
    MEM_0,
    MEM_1
} DMAC_MEMORY_t;

typedef enum
{
    BEATSIZE_1,
    BEATSIZE_4,
    BEATSIZE_8,
    BEATSIZE_16
} DMAC_NUM_BEATS_t;

typedef enum
{
    PRIO_LOW,
    PRIO_MEDIUM,
    PRIO_HIGH,
    PRIO_VERY_HIGH
} DMAC_PRIORITY_LEVEL_t;

typedef enum
{
    INCR_ENABLE,
    INCR_DISABLE
} DMAC_INCR_t;

typedef enum
{
    DMA_IS_CTRL,
    PERIPHERAL_IS_CTRL
} DMAC_DMA_FLOW_CONTROLLER_t;

typedef enum
{
    PSIZE_VAL,
    FIXED_TO_WORD
} DMAC_PERIPHERAL_INCR_OFFS_SIZE_t;

typedef enum
{
    CT_IS_MEM_0,
    CT_IS_MEM_1
} DMAC_CURRENT_TARGET_MEM_t;

typedef enum
{
    ONE_FOURTH,
    HALF,
    THREE_FOURTH,
    FULL
} DMAC_FIFO_THRESHOLD_t;

/**
 * @}
 */

// Function prototypes
extern DMAC_RETURN_CODE_t dmacInitDMAC(DMA_TypeDef *dmac);
extern DMAC_RETURN_CODE_t dmacSelectDMAC(DMA_TypeDef *dmac);
extern DMAC_RETURN_CODE_t dmacEnableStream(DMA_Stream_TypeDef *stream);
extern DMAC_RETURN_CODE_t dmacDisableStream(DMA_Stream_TypeDef *stream);
extern DMAC_RETURN_CODE_t dmacSetMemoryAddress(DMA_Stream_TypeDef *stream, DMAC_MEMORY_t memType, uint32_t addr);
extern DMAC_RETURN_CODE_t dmacSetPeripheralAddress(DMA_Stream_TypeDef *stream, uint32_t dst);

extern DMAC_RETURN_CODE_t dmacSetDataFlowDirection(DMA_Stream_TypeDef *stream, DMAC_DIRECTION_t dir);
extern DMAC_RETURN_CODE_t dmacAssignStreamAndChannel(DMA_Stream_TypeDef *stream, DMAC_CHANNEL_t chn);
extern DMAC_RETURN_CODE_t dmacSetNumData(DMA_Stream_TypeDef *stream, uint32_t numData);
extern DMAC_RETURN_CODE_t dmacSetPriorityLevel(DMA_Stream_TypeDef *stream, DMAC_PRIORITY_LEVEL_t prio);
extern DMAC_RETURN_CODE_t dmacSetMemoryIncrementMode(DMA_Stream_TypeDef *stream, DMAC_INCR_t incr);

extern DMAC_RETURN_CODE_t dmacSetPeripheralIncrementMode(DMA_Stream_TypeDef *stream, DMAC_INCR_t incr);
extern DMAC_RETURN_CODE_t dmacSetMemoryBurstBeats(DMA_Stream_TypeDef *stream, DMAC_NUM_BEATS_t beats);
extern DMAC_RETURN_CODE_t dmacSetPeripheralBurstBeats(DMA_Stream_TypeDef *stream, DMAC_NUM_BEATS_t beats);

extern DMAC_RETURN_CODE_t dmacEnableInterrupt(DMA_Stream_TypeDef *stream, DMAC_IRQ_t irqType);
extern DMAC_RETURN_CODE_t dmacEnableInterruptList(DMA_Stream_TypeDef *stream, DMAC_IRQ_t *irqList, uint8_t size);
extern DMAC_RETURN_CODE_t dmacDisableInterrupt(DMA_Stream_TypeDef *stream, DMAC_IRQ_t irqType);
extern DMAC_RETURN_CODE_t dmacDisableInterruptList(DMA_Stream_TypeDef *stream, DMAC_IRQ_t *irqList, uint8_t size);
extern uint32_t           dmacGetLowInterruptStatus(DMA_TypeDef *dma);
extern uint32_t           dmacGetHighInterruptStatus(DMA_TypeDef *dma);
extern void               dmacClearInterruptFlag(DMA_TypeDef *dma, DMA_Stream_TypeDef *stream, DMAC_IRQ_t flag);
extern void               dmacClearAllStreamIrqFlags(DMA_TypeDef *dmac, DMA_Stream_TypeDef *stream);

extern DMAC_RETURN_CODE_t dmacSelectDMAFlowController(DMA_Stream_TypeDef *stream, DMAC_DMA_FLOW_CONTROLLER_t flowCtrl);
extern DMAC_RETURN_CODE_t dmacEnableCircularMode(DMA_Stream_TypeDef *stream);
extern DMAC_RETURN_CODE_t dmacDisableCircularMode(DMA_Stream_TypeDef *stream);
extern DMAC_RETURN_CODE_t dmacConfigPeriphOffsSize(DMA_Stream_TypeDef *stream, DMAC_PERIPHERAL_INCR_OFFS_SIZE_t offset);
extern DMAC_RETURN_CODE_t dmacEnableDoubleBufferMode(DMA_Stream_TypeDef *stream);
extern DMAC_RETURN_CODE_t dmacDisableDoubleBufferMode(DMA_Stream_TypeDef *stream);

extern DMAC_RETURN_CODE_t dmacSelectMem2MemTarget(DMA_Stream_TypeDef *stream, DMAC_CURRENT_TARGET_MEM_t mem);
extern DMAC_RETURN_CODE_t dmacEnableFifoMode(DMA_Stream_TypeDef *stream);
extern DMAC_RETURN_CODE_t dmacDisableFifoMode(DMA_Stream_TypeDef *stream);
extern DMAC_RETURN_CODE_t dmacSetFifoThreshold(DMA_Stream_TypeDef *stream, DMAC_FIFO_THRESHOLD_t th);
extern DMAC_RETURN_CODE_t dmacSetPeripheralDataFormat(DMA_Stream_TypeDef *stream, DMAC_DATA_FORMAT_t format);
extern DMAC_RETURN_CODE_t dmacSetMemoryDataFormat(DMA_Stream_TypeDef *stream, DMAC_DATA_FORMAT_t format);

// Deprecated function
extern DMAC_RETURN_CODE_t dmacSetStreamAndChannel(DMA_Stream_TypeDef *stream,
                                                  DMAC_CHANNEL_t channel,
                                                  uint32_t src,
                                                  uint32_t dest,
                                                  uint16_t numData,
                                                  DMAC_DATA_FORMAT_t srcFormat,
                                                  DMAC_DATA_FORMAT_t dstFormat,
                                                  DMAC_DIRECTION_t dir,
                                                  DMAC_IRQ_t irqType);
extern DMAC_RETURN_CODE_t dmacSetDataFormat(DMA_Stream_TypeDef *stream, DMAC_DATA_FORMAT_t format);

#ifdef __cplusplus
}
#endif


#endif /* MCALDMAC_H_ */
