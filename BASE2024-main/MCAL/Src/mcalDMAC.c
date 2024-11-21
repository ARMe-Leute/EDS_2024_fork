/**
 * @defgroup dma  DMA Functions (mcalDMAC.h/.c)
 * @defgroup dma0 Deprecated DMA functions
 * @ingroup  dma
 * @defgroup dma1 Basic DMA functions
 * @ingroup  dma
 * @defgroup dma2 DMA Stream Configuration
 * @ingroup  dma
 * @defgroup dma3 DMA Interrupt Functions
 * @ingroup  dma
 * @defgroup dma4 DMA Buffer/FIFO Functions
 * @ingroup  dma
 * @defgroup dma5 DMA Enumerations and Definitions
 * @ingroup  dma
 *
 * @file        mcalDMAC.c
 * @brief       mcalDMAC.c is part of the MCAL library for STM32F4xx.
 * @author      Dipl.-Ing. Ralf Jesse (embedded@ralf-jesse.de)
 * @date        Nov. 11, 2020
 *
 * @version     0.1
 * @copyright   GNU Public License Wersion 3 (GPLv3)
 *
 * @note
 * This module is still under construction. It is far away from being finished. However, functions that are marked as<br>
 * <b>deprecated</b> will not be removed from the module.
 *
 * This program is free software: you can redistribute it and/or modify it under the terms of the GNU General Public License<br>
 * as published by the Free Software Foundation, either version 3 of the License, or any later version.<br>
 *
 * This program is distributed in the hope that it will be useful, but WITHOUT ANY WARRANTY; without even the implied warranty<br>
 * of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the GNU General Public License for more details.<br>
 *
 * You should have received a copy of the GNU General Public License along with this program. If not, see <http://www.gnu.org/licenses/>.
 */

#include <mcalDMAC.h>

/**
 * Verifies the integrity of the direction.
 *
 * @param dir : Must be in [MEM_2_PER, PER_2_MEM, MEM_2_MEM]
 */
static bool dmacVerifyDirection(DMAC_DIRECTION_t dir)
{
    if ((PER_2_MEM == dir) || (MEM_2_PER == dir) || (MEM_2_MEM == dir))
    {
        return true;
    }

    return false;
}

/**
 * Verifies the integrity of the data format.
 *
 * @param format : Must be in [BYTE, HALFWORD, WORD]
 */
static bool dmacVerifyDataFormat(DMAC_DATA_FORMAT_t format)
{
    if ((BYTE == format) || (HALFWORD == format) || (WORD == format))
    {
        return true;
    }

    return false;
}

/**
 * Verifies the integrity of the interrupt type.
 *
 * @param irq : Must be in [DIRECT_MODE_ERR, TX_ERR, TX_HALF, TX_COMPLETE]
 */
static bool dmacVerifyIrqType(DMAC_IRQ_t irq)
{
    if ((DIRECT_MODE_ERR == irq) || (TX_ERR == irq) || (TX_HALF == irq) ||
        (TX_COMPLETE     == irq) || (FIFO_ERROR_ENABLE == irq))
    {
        return true;
    }

    return false;
}

/**
 * Verifies the integrity of the data block size.
 *
 * @param   numData : 0 < numData < 65536
 */
static bool dmacVerifyNumData(uint16_t numData)
{
    if (numData < DMA_MAX_DATA_LEN)
    {
        return true;
    }

    return false;
}

/**
 * Verifies the integrity of the channel number
 *
 * @param chn : The channel of the DMA stream
 */
static bool dmacVerifyChannelNum(DMAC_CHANNEL_t chn)
{
    if ((CHANNEL_0 != chn) && (CHANNEL_1 != chn) && (CHANNEL_2 != chn) && (CHANNEL_3 != chn) &&
        (CHANNEL_4 != chn) && (CHANNEL_5 != chn) && (CHANNEL_6 != chn) && (CHANNEL_7 != chn))
    {
        return false;
    }
    return true;
}

/**
 * Verifies the integrity of the priority level
 *
 * @param  prio : The selected priority level
 */
static bool dmacVerifyPriorityLevel(DMAC_PRIORITY_LEVEL_t prio)
{
    if ((PRIO_LOW != prio) && (PRIO_MEDIUM != prio) && (PRIO_HIGH != prio) && (PRIO_VERY_HIGH != prio))
    {
        return false;
    }
    return true;
}

/**
 * Verifies the integrity of the incrementation mode for both, memory and peripheral.
 */
static bool dmacVerifyIncrementMode(DMAC_INCR_t mode)
{
    if ((INCR_ENABLE != mode) && (INCR_DISABLE != mode))
    {
        return false;
    }
    return true;
}

/**
 * Verifies the number of beats which can be defined to unload the AHB
 */
static bool dmacVerifyNumBeats(DMAC_NUM_BEATS_t beats)
{
    if ((BEATSIZE_1 != beats) && (BEATSIZE_4 != beats) && (BEATSIZE_8 != beats) && (BEATSIZE_16 != beats))
    {
        return false;
    }
    return true;
}

/**
 * Verifies the validity of the DMA flow controller.
 */
static bool dmacVerifyDMAFlowController(DMAC_DMA_FLOW_CONTROLLER_t flowCtrl)
{
    if ((DMA_IS_CTRL != flowCtrl) && (PERIPHERAL_IS_CTRL != flowCtrl))
    {
        return false;
    }
    return true;
}

/**
 * Verifies the integrity of the PINCOS offset.
 */
static bool dmacVerifyPeripheralOffsetSize(DMAC_PERIPHERAL_INCR_OFFS_SIZE_t offset)
{
    if ((PSIZE_VAL != offset) && (FIXED_TO_WORD != offset))
    {
        return false;
    }
    return true;
}

/**
 * Verifies the desired target memory.
 */
static bool dmacVerifyCurrentTargetMem(DMAC_CURRENT_TARGET_MEM_t mem)
{
    if ((CT_IS_MEM_0 != mem) && (CT_IS_MEM_1 != mem))
    {
        return false;
    }
    return true;
}

/**
 * Verifies the validity of the FIFO threshold level.
 */
static bool dmacVerifyFifoThresholdLvl(DMAC_FIFO_THRESHOLD_t th)
{
    if ((ONE_FOURTH != th) && (HALF != th) && (THREE_FOURTH != th) && (FULL != th))
    {
        return false;
    }
    return true;
}

/**
 * @deprecated
 * @ingroup dma0
 * Don't use this function any longer. Use DMAC_RETURN_CODE_t dmacSelectDMAC(DMA_TypeDef *dmac) instead.
 *
 * @param   *dmac : Pointer to the DMA controller
 * @return  DMAC_RETURN_CODE_t
 *
 * <br>
 * <b>Affected register and bit(s)</b><br>
 * <table>
 *      <tr>
 *          <th>Register</th>
 *          <th>Bit name</th>
 *          <th>Bit(s)</th>
 *      </tr>
 *      <tr>
 *          <td>AHB1ENR</td>
 *          <td rowspan="1">DMA1 / DMA2</td>
 *          <td rowspan="1">21 / 22</td>
 *      </tr>
 * </table>
 */
DMAC_RETURN_CODE_t dmacInitDMAC(DMA_TypeDef *dmac)
{
    if (DMA1 == dmac)
    {
        RCC->AHB1ENR |= RCC_AHB1ENR_DMA1EN;
    }
    else if (DMA2 == dmac)
    {
        RCC->AHB1ENR |= RCC_AHB1ENR_DMA2EN;
    }

    return DMAC_OK;
}

/**
 * @ingroup dma1
 * This function turns the bus clock of the given DMA controller on. It verifies the validity of the given DMA controller.
 *
 * @param   *dmac : Pointer to the DMA controller
 * @return  DMAC_RETURN_CODE_t
 *
 * <br>
 * <b>Affected register and bit(s)</b><br>
 * <table>
 *      <tr>
 *          <th>Register</th>
 *          <th>Bit name</th>
 *          <th>Bit(s)</th>
 *      </tr>
 *      <tr>
 *          <td>AHB1ENR</td>
 *          <td rowspan="1">DMA1 / DMA2</td>
 *          <td rowspan="1">21 / 22</td>
 *      </tr>
 * </table>
 */
DMAC_RETURN_CODE_t dmacSelectDMAC(DMA_TypeDef *dmac)
{
    if (DMA1 == dmac)
    {
        RCC->AHB1ENR |= RCC_AHB1ENR_DMA1EN;
    }
    else if (DMA2 == dmac)
    {
        RCC->AHB1ENR |= RCC_AHB1ENR_DMA2EN;
    }

    return DMAC_OK;
}


/**
 * @ingroup dma1
 * Enables the stream. With verification of the stream parameter.
 *
 * @param  *stream : Pointer to the DMA stream
 *
 * @note
 * The registers of a DMA stream cannot be configured when it is enabled. Always enable a DMA stream only **AFTER** configuration!
 *
 * <br>
 * <b>Affected register and bit(s)</b><br>
 * <table>
 *      <tr>
 *          <th>Register</th>
 *          <th>Bit name</th>
 *          <th>Bit(s)</th>
 *      </tr>
 *      <tr>
 *          <td>SxCR</td>
 *          <td rowspan="1">EN</td>
 *          <td rowspan="1">0</td>
 *      </tr>
 * </table>
 */
DMAC_RETURN_CODE_t dmacEnableStream(DMA_Stream_TypeDef *stream)
{
    stream->CR |= DMA_SxCR_EN;
    return DMAC_OK;
}

/**
 * @ingroup dma1
 * Disables the stream. It verifies the desired stream parameter. This function checks the EN bit and returns only when the stream
 * is disabled (EN = 0)!
 *
 * @param    *stream : Pointer to the DMA stream
 * @return   DMAC_RETURN_CODE_t
 *
 * @note
 * It is required to disable a DMA stream **BEFORE** writing new settings to the registers.
 *
 * <br>
 * <b>Affected register and bit(s)</b><br>
 * <table>
 *      <tr>
 *          <th>Register</th>
 *          <th>Bit name</th>
 *          <th>Bit(s)</th>
 *      </tr>
 *      <tr>
 *          <td>SxCR</td>
 *          <td rowspan="1">EN</td>
 *          <td rowspan="1">0</td>
 *      </tr>
 * </table>
 */
DMAC_RETURN_CODE_t dmacDisableStream(DMA_Stream_TypeDef *stream)
{
    stream->CR &= ~DMA_SxCR_EN_Msk;
    while (stream->CR & DMA_SxCR_EN)
    {
        // Wait until the EN bit of the stream is "0"
    }

    return DMAC_OK;
}

/**
 * @ingroup dma0
 * This multi-param function has been split up into separate functions. The following table shows the names of the replacement<br>
 * functions.<br>
 *
 * <table>
 *      <caption id="multi_row">SF: Separate functions</caption>
 *      <tr>
 *          <th>Parameter(s)</th>
 *          <th>New function</th>
 *      </tr>
 *      <tr>
 *          <td rowspan="1">stream + channel</td>
 *          <td>dmacSetStreamAndChannel()</td>
 *      </tr>
 *      <tr>
 *          <td rowspan="1">source + destination address</td>
 *          <td>dmacSetMemoryAddress() and dmacSetPeripheralAddress()</td>
 *      </tr>
 *      <tr>
 *          <td rowspan="1">numData</td>
 *          <td>dmacSetNumData()</td>
 *      </tr>
 *      <tr>
 *          <td rowspan="1">Data format (src and dst)</td>
 *          <td>dmacSetDataFormat()</td>
 *      </tr>
 *      <tr>
 *          <td rowspan="1">dir</td>
 *          <td>dmacSetDataFlowDirection()</td>
 *      </tr>
 *      <tr>
 *          <td rowspan="1">irq</td>
 *          <td>dmacEnableInterrupt(), dmacEnableInterruptList(), dmacDisableInterrupt(), dmacDisableInterruptList()</td>
 *      </tr>
 * </table>
 *
 * Configures the selected DMA stream.
 *
 * @param   *stream     : Pointer to the stream
 * @param    chn        : Channel
 * @param    src        : Address of the data source
 * @param    dest       : Address of the data destination
 * @param    numData    : Number of the data
 * @param    srcFormat  : Format of the source data size
 * @param    dstFormat  : Format of the destination data size
 * @param    dir        : Direction of the data transfer
 * @param    irqType    : Interrupt types
 *
 * @return  DMAC_RETURN_CODE_t
 *
 * @note
 * Before calling this function the given stream must be disabled. After a successful return don't forget to<br>
 *    + enable the DMA stream<br>
 *    + add the stream interrupt to NVIC<br>
 *
 * @note
 * Although this function uses many parameters it does not support all DMA settings. E.g. this function does not support<br>
 * FIFO functionality, and it does not let the user select the incrementation mode of memory nor peripherals.<br>
 */
DMAC_RETURN_CODE_t dmacSetStreamAndChannel(DMA_Stream_TypeDef *stream,
                                           DMAC_CHANNEL_t chn,
                                           uint32_t src,
                                           uint32_t dest,
                                           uint16_t numData,
                                           DMAC_DATA_FORMAT_t srcFormat,
                                           DMAC_DATA_FORMAT_t dstFormat,
                                           DMAC_DIRECTION_t dir,
                                           DMAC_IRQ_t irqType)
{
    // Verify the validity of the given parameters
    if (dmacVerifyDirection(dir) != true)
    {
        return DMAC_INVALID_DIR;
    }
    if (dmacVerifyNumData(numData) != true)
    {
        return DMAC_INVALID_DATA_LEN;
    }
    if (dmacVerifyDataFormat(srcFormat) != true)
    {
        return DMAC_INVALID_SRC_DATA_FORMAT;
    }
    if (dmacVerifyDataFormat(dstFormat) != true)
    {
        return DMAC_INVALID_DEST_DATA_FORMAT;
    }
    if (dmacVerifyIrqType(irqType) != true)
    {
        return DMAC_INVALID_IRQ_TYPE;
    }

    // All parameter verfication passed with success. We can now continue
    // with the setup of the DMA stream.
    stream->CR    = chn << DMA_SxCR_CHSEL_Pos;          // Set the channel
    stream->CR   |= irqType;                            // OR-ed interrupt types
    stream->CR   &= ~DMA_SxCR_MSIZE_Msk;                // Reset MSIZE
    stream->CR   |= srcFormat << DMA_SxCR_MSIZE_Pos;    // Set MSIZE to srcFormat
    stream->CR   &= ~DMA_SxCR_PSIZE_Msk;                // Reset PSIZE
    stream->CR   |= dstFormat << DMA_SxCR_PSIZE_Pos;    // Set PSIZE to dstFormat
    stream->CR   &= ~DMA_SxCR_DIR_Msk;                  // Reset DIR
    stream->CR   |= dir << DMA_SxCR_DIR_Pos;            // Set DIR to dir
    stream->CR   |= DMA_SxCR_MINC;                      // Always use auto-increment

    stream->M0AR  = src;                                // Set address of the data source
    stream->PAR   = dest;                               // Set address of the destination
    stream->NDTR  = numData;                            // Set size of data block

    return DMAC_OK;
}

/**
 * @ingroup dma2
 * Assigns a channel to a DMA stream
 *
 * <b>Affected register/bit</b>
 *
 * @param  *stream : Pointer to the DMAC
 * @param   chn    : Number of the channel
 * @return  DMAC_RETURN_CODE_t
 *
 * @note
 * This function must be called at first function after dmacDisableStream(). Reason: DMA stream and channel are directly assigned<br>
 * to the control register. If you call this function after setting other DMA parameters that settings will be overwritten.
 *
 * <br>
 * <b>Affected register and bit(s)</b><br>
 * <table>
 *      <tr>
 *          <th>Register</th>
 *          <th>Bit name</th>
 *          <th>Bit(s)</th>
 *      </tr>
 *      <tr>
 *          <td>SxCR</td>
 *          <td rowspan="1">CHSEL</td>
 *          <td rowspan="1">27...25</td>
 *      </tr>
 * </table>
 */
DMAC_RETURN_CODE_t dmacAssignStreamAndChannel(DMA_Stream_TypeDef *stream, DMAC_CHANNEL_t chn)
{
    if (dmacVerifyChannelNum(chn) != true)
    {
        return DMAC_INVALID_CHANNEL;
    }
    stream->CR = chn << DMA_SxCR_CHSEL_Pos;
    return DMAC_OK;
}


/**
 * @ingroup dma2
 * Sets the register M0AR/M1AR depending on param 2.
 *
 * @param  *stream  : Pointer to the DMA stream
 * @param   memType : Memory type (MEM_0 or MEM_1)
 * @param   addr    : Address of the memory type
 * @return DMAC_RETURN_CODE_t
 *
 * @note
 * This function must be called twice in the case of a memory-to-memory transfer.
 *
 * <br>
 * <b>Affected register and bit(s)</b><br>
 * <table>
 *      <tr>
 *          <th>Register</th>
 *          <th>Bit name</th>
 *          <th>Bit(s)</th>
 *      </tr>
 *      <tr>
 *          <td>M0AR/M1AR</td>
 *          <td rowspan="1">MADDR</td>
 *          <td rowspan="1">31..0</td>
 *      </tr>
 * </table>
 */
DMAC_RETURN_CODE_t dmacSetMemoryAddress(DMA_Stream_TypeDef *stream, DMAC_MEMORY_t memType, uint32_t addr)
{
    if (MEM_0 == memType)
    {
        stream->M0AR = addr;
    }
    else if (MEM_1 == memType)
    {
        stream->M1AR = addr;
    }
    return DMAC_OK;
}

/**
 * @ingroup dma2
 * Sets the address of the peripheral register.
 *
 * @param  *stream : Pointer to the DMA stream
 * @param   addr   : Address (source or destination) of the peripheral register
 * @return DMAC_RETURN_CODE_t
 *
 * <br>
 * <b>Affected register and bit(s)</b><br>
 * <table>
 *      <tr>
 *          <th>Register</th>
 *          <th>Bit name</th>
 *          <th>Bit(s)</th>
 *      </tr>
 *      <tr>
 *          <td>PAR</td>
 *          <td rowspan="1">PADDR</td>
 *          <td rowspan="1">31...0</td>
 *      </tr>
 * </table>
 */
DMAC_RETURN_CODE_t dmacSetPeripheralAddress(DMA_Stream_TypeDef *stream, uint32_t addr)
{
    stream->PAR = addr;
    return DMAC_OK;
}

/**
 * @ingroup dma2
 * Sets the direction of the data transfer.
 *
 * @param  *stream : Pointer to the DMA stream
 * @param   dir    : Direction of the data flow (PER_2_MEM, MEM_2_PER, MEM_2_MEM)
 * @return DMAC_RETURN_CODE_t
 *
 * <br>
 * <b>Affected register and bit(s)</b><br>
 * <table>
 *      <tr>
 *          <th>Register</th>
 *          <th>Bit name</th>
 *          <th>Bit(s)</th>
 *      </tr>
 *      <tr>
 *          <td>SxCR</td>
 *          <td rowspan="1">DIR</td>
 *          <td rowspan="1">7...6</td>
 *      </tr>
 * </table>
 */
DMAC_RETURN_CODE_t dmacSetDataFlowDirection(DMA_Stream_TypeDef *stream, DMAC_DIRECTION_t dir)
{
    if (dmacVerifyDirection(dir) != true)
    {
        return DMAC_INVALID_DIR;
    }

    stream->CR   &= ~DMA_SxCR_DIR_Msk;                  // Reset DIR
    stream->CR   |= dir << DMA_SxCR_DIR_Pos;            // Set DIR to dir
    return DMAC_OK;
}

/**
 * @ingroup dma2
 * Sets the number of data that shall be transferred.
 *
 * @param   *stream  : Pointer to the DMA stream
 * @param    numData : Number of data that shall be transferred
 * @return  DMAC_RETURNCODE_t
 *
 * <br>
 * <b>Affected register and bit(s)</b><br>
 * <table>
 *      <tr>
 *          <th>Register</th>
 *          <th>Bit name</th>
 *          <th>Bit(s)</th>
 *      </tr>
 *      <tr>
 *          <td>NDTR</td>
 *          <td rowspan="1">Number of data</td>
 *          <td rowspan="1">15...0</td>
 *      </tr>
 * </table>
 */
DMAC_RETURN_CODE_t dmacSetNumData(DMA_Stream_TypeDef *stream, uint32_t numData)
{
    if (dmacVerifyNumData(numData) != true)
    {
        return DMAC_INVALID_DATA_LEN;
    }
    stream->NDTR = numData;
    return DMAC_OK;
}

/**
 * @ingroup dma0
 * Sets the format of the data that shall be transferred. The data format is the same for both the source and the destination.
 *
 * @param   *stream : Pointer to the DMA stream
 * @param    format : Data format of the data
 * @return  DMAC_RETURN_CODE_t
 *
 * @note
 * This function shall not be used any longer since it sets the data format (byte, half-word, or word) to the same value.<br>
 * However, it might be possible that the data format of any peripheral is different from the data format of the memory.<br>
 * Use <b>dmacSetPeripheralDataFormat()</b> and <b>dmacSetMemoryDataFormat()</b> instead.
 *
 * <br>
 * <b>Affected register and bit(s)</b><br>
 * <table>
 *      <tr>
 *          <th>Register</th>
 *          <th>Bit name</th>
 *          <th>Bit(s)</th>
 *      </tr>
 *      <tr>
 *          <td>SxCR</td>
 *          <td rowspan="1">MSIZE</td>
 *          <td rowspan="1">14...13</td>
 *      </tr>
 *      <tr>
 *          <td>SxCR</td>
 *          <td>PSIZE</td>
 *          <td>12...11</td>
 *      </tr>
 * </table>
 */
DMAC_RETURN_CODE_t dmacSetDataFormat(DMA_Stream_TypeDef *stream, DMAC_DATA_FORMAT_t format)
{
    if (dmacVerifyDataFormat(format) != true)
    {
        return DMAC_INVALID_DATA_FORMAT;
    }
    stream->CR   &= ~DMA_SxCR_MSIZE_Msk;             // Reset MSIZE
    stream->CR   |= format << DMA_SxCR_MSIZE_Pos;    // Set MSIZE to srcFormat
    stream->CR   &= ~DMA_SxCR_PSIZE_Msk;             // Reset PSIZE
    stream->CR   |= format << DMA_SxCR_PSIZE_Pos;    // Set PSIZE to dstFormat
    return DMAC_OK;
}

/**
 * @ingroup dma2
 * Sets the data format for the peripheral device.
 *
 * @param   *stream : Pointer to the DMA stream
 * @param    format : Data format of the data
 * @return  DMAC_RETURN_CODE_t
 *
 * @note
 * This function replaces the deprecated function <b>dmacSetDataFormat()</b>. To set the data format of the memory you need<br>
 * to use the counter function <b>dmacSetMemoryDataFormat()</b>, too.
 *
 * <br>
 * <b>Affected register and bit(s)</b><br>
 * <table>
 *      <tr>
 *          <th>Register</th>
 *          <th>Bit name</th>
 *          <th>Bit(s)</th>
 *      </tr>
 *      <tr>
 *          <td>SxCR</td>
 *          <td rowspan="1">PSIZE</td>
 *          <td rowspan="1">12...11</td>
 *      </tr>
 * </table>
 */
DMAC_RETURN_CODE_t dmacSetPeripheralDataFormat(DMA_Stream_TypeDef *stream, DMAC_DATA_FORMAT_t format)
{
    if (dmacVerifyDataFormat(format) != true)
    {
        return DMAC_INVALID_DATA_FORMAT;
    }
    stream->CR   &= ~DMA_SxCR_PSIZE_Msk;             // Reset PSIZE
    stream->CR   |= format << DMA_SxCR_PSIZE_Pos;    // Set PSIZE to dstFormat
    return DMAC_OK;
}

/**
 * @ingroup dma2
 * Sets the data format for the memory.
 *
 * @param   *stream : Pointer to the DMA stream
 * @param    format : Data format of the data
 * @return  DMAC_RETURN_CODE_t
 *
 * @note
 * This function replaces the deprecated function <b>dmacSetDataFormat()</b>. To set the data format of the peripheral you need<br>
 * to use the counter function <b>dmacSetPeripheralDataFormat()</b>, too.
 *
 * <br>
 * <b>Affected register and bit(s)</b><br>
 * <table>
 *      <tr>
 *          <th>Register</th>
 *          <th>Bit name</th>
 *          <th>Bit(s)</th>
 *      </tr>
 *      <tr>
 *          <td>SxCR</td>
 *          <td rowspan="1">MSIZE</td>
 *          <td rowspan="1">14...13</td>
 *      </tr>
 * </table>
 */
DMAC_RETURN_CODE_t dmacSetMemoryDataFormat(DMA_Stream_TypeDef *stream, DMAC_DATA_FORMAT_t format)
{
    if (dmacVerifyDataFormat(format) != true)
    {
        return DMAC_INVALID_DATA_FORMAT;
    }
    stream->CR   &= ~DMA_SxCR_MSIZE_Msk;             // Reset MSIZE
    stream->CR   |= format << DMA_SxCR_MSIZE_Pos;    // Set MSIZE to srcFormat
    return DMAC_OK;
}

/**
 * @ingroup dma3
 * This function clears all interrupt flags of the given DMA and stream combination.
 *
 * @param   *dmac   : Pointer to the DMA controller (DMA1 or DMA2)
 * @param   *stream : Pointer to the DMA stream
 *
 * <br>
 * <b>Affected register and bit(s)</b><br>
 * <table>
 *      <tr>
 *          <th>Register</th>
 *          <th>Bit name</th>
 *          <th>Bit(s)</th>
 *      </tr>
 *      <tr>
 *          <td>LIFCR</td>
 *          <td rowspan="1">Different interrupts</td>
 *          <td rowspan="1">31...0</td>
 *      </tr>
 *      <tr>
 *          <td>HIFCR</td>
 *          <td rowspan="1">Different interrupts</td>
 *          <td rowspan="1">31...0</td>
 *      </tr>
 * </table>
 */
void dmacClearAllStreamIrqFlags(DMA_TypeDef *dmac, DMA_Stream_TypeDef *stream)
{
    // Reset all interrupt flags for DMA_Stream0
    if ((DMA1_Stream0 == stream) || (DMA2_Stream0 == stream))
    {
        dmac->LIFCR |= (DMA_LIFCR_CFEIF0 | DMA_LIFCR_CDMEIF0 | DMA_LIFCR_CTEIF0 | DMA_LIFCR_CHTIF0 | DMA_LIFCR_CTCIF0);
    }

    // Reset all interrupt flags for DMAx_Stream1
    if ((DMA1_Stream1 == stream) || (DMA2_Stream1 == stream))
    {
        dmac->LIFCR |= (DMA_LIFCR_CFEIF1 | DMA_LIFCR_CDMEIF1 | DMA_LIFCR_CTEIF1 | DMA_LIFCR_CHTIF1 | DMA_LIFCR_CTCIF1);
    }

    // Reset all interrupt flags for DMA_Stream2
    if ((DMA1_Stream2 == stream) || (DMA2_Stream2 == stream))
    {
        dmac->LIFCR |= (DMA_LIFCR_CFEIF2 | DMA_LIFCR_CDMEIF2 | DMA_LIFCR_CTEIF2 | DMA_LIFCR_CHTIF2 | DMA_LIFCR_CTCIF2);
    }

    // Reset all interrupt flags for DMA_Stream3
    if ((DMA1_Stream3 == stream) || (DMA2_Stream3 == stream))
    {
        dmac->LIFCR |= (DMA_LIFCR_CFEIF3 | DMA_LIFCR_CDMEIF3 | DMA_LIFCR_CTEIF3 | DMA_LIFCR_CHTIF3 | DMA_LIFCR_CTCIF3);
    }

    // Reset all interrupt flags for DMA_Stream4
    if ((DMA1_Stream4 == stream) || (DMA2_Stream4 == stream))
    {
        dmac->HIFCR |= (DMA_HIFCR_CFEIF4 | DMA_HIFCR_CDMEIF4 | DMA_HIFCR_CTEIF4 | DMA_HIFCR_CHTIF4 | DMA_HIFCR_CTCIF4);
    }

    // Reset all interrupt flags for DMA_Stream5
    if ((DMA1_Stream5 == stream) || (DMA2_Stream5 == stream))
    {
        dmac->HIFCR |= (DMA_HIFCR_CFEIF5 | DMA_HIFCR_CDMEIF5 | DMA_HIFCR_CTEIF5 | DMA_HIFCR_CHTIF5 | DMA_HIFCR_CTCIF5);
    }

    // Reset all interrupt flags for DMA_Stream6
    if ((DMA1_Stream6 == stream) || (DMA2_Stream6 == stream))
    {
        dmac->HIFCR |= (DMA_HIFCR_CFEIF6 | DMA_HIFCR_CDMEIF6 | DMA_HIFCR_CTEIF6 | DMA_HIFCR_CHTIF6 | DMA_HIFCR_CTCIF6);
    }

    // Reset all interrupt flags for DMA_Stream7
    if ((DMA1_Stream7 == stream) || (DMA2_Stream7 == stream))
    {
        dmac->HIFCR |= (DMA_HIFCR_CFEIF7 | DMA_HIFCR_CDMEIF7 | DMA_HIFCR_CTEIF7 | DMA_HIFCR_CHTIF7 | DMA_HIFCR_CTCIF7);
    }
}

/**
 * @ingroup dma3
 * Clears the desired interrupt flag of the desired DMA stream.
 *
 * @param   *dma    : Pointer to the DMA controller (DMA1 or DMA2)
 * @param   *stream : Pointer to the DMA stream
 * @param    flag   : A single interrupt flag
 *
 * <br>
 * <b>Affected register and bit(s)</b><br>
 * <table>
 *      <tr>
 *          <th>Register</th>
 *          <th>Bit name</th>
 *          <th>Bit(s)</th>
 *      </tr>
 *      <tr>
 *          <td>LIFCR or HIFCR</td>
 *          <td rowspan="1">Different interrupts</td>
 *          <td rowspan="1">see Reference Manual</td>
 *      </tr>
 * </table>
 */
void dmacClearInterruptFlag(DMA_TypeDef *dma, DMA_Stream_TypeDef *stream, DMAC_IRQ_t flag)
{
    if ((DMA1_Stream0 == stream) || (DMA2_Stream0 == stream))
    {
        switch (flag)
        {
            case FIFO_ERROR_ENABLE:
                dma->LIFCR |= DMA_LIFCR_CFEIF0;
                break;

            case DIRECT_MODE_ERR:
                dma->LIFCR |= DMA_LIFCR_CDMEIF0;
                break;

            case TX_ERR:
                dma->LIFCR |= DMA_LIFCR_CTEIF0;
                break;

            case TX_HALF:
                dma->LIFCR |= DMA_LIFCR_CHTIF0;
                break;

            case TX_COMPLETE:
                dma->LIFCR |= DMA_LIFCR_CTCIF0;
                break;

        }
    }
    else if ((DMA1_Stream1 == stream) || (DMA2_Stream1 == stream))
    {
        switch (flag)
        {
            case FIFO_ERROR_ENABLE:
                dma->LIFCR |= DMA_LIFCR_CFEIF1;
                break;

            case DIRECT_MODE_ERR:
                dma->LIFCR |= DMA_LIFCR_CDMEIF1;
                break;

            case TX_ERR:
                dma->LIFCR |= DMA_LIFCR_CTEIF1;
                break;

            case TX_HALF:
                dma->LIFCR |= DMA_LIFCR_CHTIF1;
                break;

            case TX_COMPLETE:
                dma->LIFCR |= DMA_LIFCR_CTCIF1;
                break;
        }
    }
    else if ((DMA1_Stream2 == stream) || (DMA2_Stream2 == stream))
    {
        switch (flag)
        {
            case FIFO_ERROR_ENABLE:
                dma->LIFCR |= DMA_LIFCR_CFEIF2;
                break;

            case DIRECT_MODE_ERR:
                dma->LIFCR |= DMA_LIFCR_CDMEIF2;
                break;

            case TX_ERR:
                dma->LIFCR |= DMA_LIFCR_CTEIF2;
                break;

            case TX_HALF:
                dma->LIFCR |= DMA_LIFCR_CHTIF2;
                break;

            case TX_COMPLETE:
                dma->LIFCR |= DMA_LIFCR_CTCIF2;
                break;
        }
    }
    else if ((DMA1_Stream3 == stream) || (DMA2_Stream3 == stream))
    {
        switch (flag)
        {
            case FIFO_ERROR_ENABLE:
                dma->LIFCR |= DMA_LIFCR_CFEIF3;
                break;

            case DIRECT_MODE_ERR:
                dma->LIFCR |= DMA_LIFCR_CDMEIF3;
                break;

            case TX_ERR:
                dma->LIFCR |= DMA_LIFCR_CTEIF3;
                break;

            case TX_HALF:
                dma->LIFCR |= DMA_LIFCR_CHTIF3;
                break;

            case TX_COMPLETE:
                dma->LIFCR |= DMA_LIFCR_CTCIF3;
                break;
        }
    }
    else if ((DMA1_Stream4 == stream) || (DMA2_Stream4 == stream))
    {
        switch (flag)
        {
            case FIFO_ERROR_ENABLE:
                dma->HIFCR |= DMA_HIFCR_CFEIF4;
                break;

            case DIRECT_MODE_ERR:
                dma->HIFCR |= DMA_HIFCR_CDMEIF4;
                break;

            case TX_ERR:
                dma->HIFCR |= DMA_HIFCR_CTEIF4;
                break;

            case TX_HALF:
                dma->HIFCR |= DMA_HIFCR_CHTIF4;
                break;

            case TX_COMPLETE:
                dma->HIFCR |= DMA_HIFCR_CTCIF4;
                break;
        }
    }
    else if ((DMA1_Stream5 == stream) || (DMA2_Stream5 == stream))
    {
        switch (flag)
        {
            case FIFO_ERROR_ENABLE:
                dma->HIFCR |= DMA_HIFCR_CFEIF5;
                break;

            case DIRECT_MODE_ERR:
                dma->HIFCR |= DMA_HIFCR_CDMEIF5;
                break;

            case TX_ERR:
                dma->HIFCR |= DMA_HIFCR_CTEIF5;
                break;

            case TX_HALF:
                dma->HIFCR |= DMA_HIFCR_CHTIF5;
                break;

            case TX_COMPLETE:
                dma->HIFCR |= DMA_HIFCR_CTCIF5;
                break;
        }
    }
    else if ((DMA1_Stream6 == stream) || (DMA2_Stream6 == stream))
    {
        switch (flag)
        {
            case FIFO_ERROR_ENABLE:
                dma->HIFCR |= DMA_HIFCR_CFEIF6;
                break;

            case DIRECT_MODE_ERR:
                dma->HIFCR |= DMA_HIFCR_CDMEIF6;
                break;

            case TX_ERR:
                dma->HIFCR |= DMA_HIFCR_CTEIF6;
                break;

            case TX_HALF:
                dma->HIFCR |= DMA_HIFCR_CHTIF6;
                break;

            case TX_COMPLETE:
                dma->HIFCR |= DMA_HIFCR_CTCIF6;
                break;
        }
    }
    else if ((DMA1_Stream7 == stream) || (DMA2_Stream7 == stream))
    {
        switch (flag)
        {
            case FIFO_ERROR_ENABLE:
                dma->HIFCR |= DMA_HIFCR_CFEIF7;
                break;

            case DIRECT_MODE_ERR:
                dma->HIFCR |= DMA_HIFCR_CDMEIF7;
                break;

            case TX_ERR:
                dma->HIFCR |= DMA_HIFCR_CTEIF7;
                break;

            case TX_HALF:
                dma->HIFCR |= DMA_HIFCR_CHTIF7;
                break;

            case TX_COMPLETE:
                dma->HIFCR |= DMA_HIFCR_CTCIF7;
                break;
        }
    }
}

/**
 * @ingroup dma2
 * @brief Enables or disables the automatic incrementation for memory access
 *
 * @param  *stream : Pointer to the DMA stream
 * @param   incr   : Valid parameters are [INCR_ENABLE, INCR_DISABLE]
 * @return DMAC_RETURN_CODE_t
 *
 * <br>
 * <b>Affected register and bit(s)</b><br>
 * <table>
 *      <tr>
 *          <th>Register</th>
 *          <th>Bit name</th>
 *          <th>Bit(s)</th>
 *      </tr>
 *      <tr>
 *          <td>SxCR</td>
 *          <td rowspan="1">MINC</td>
 *          <td rowspan="1">10</td>
 *      </tr>
 * </table>
 */
DMAC_RETURN_CODE_t dmacSetMemoryIncrementMode(DMA_Stream_TypeDef *stream, DMAC_INCR_t incr)
{
    if (dmacVerifyIncrementMode(incr) != true)
    {
        return DMAC_INVALID_INCREMENT;
    }

    stream->CR &= ~DMA_SxCR_MINC_Msk;   // Reset previous setting
    switch (incr)
    {
        case INCR_ENABLE:
            stream->CR |= DMA_SxCR_MINC;
            break;
        case INCR_DISABLE:
            stream->CR &= ~DMA_SxCR_MINC_Msk;
            break;
        default:
            stream->CR &= ~DMA_SxCR_MINC_Msk;
            break;
    }
    return DMAC_OK;
}

/**
 * @ingroup dma2
 * @brief Enables or disables the automatic incrementation for peripheral access
 *
 * @param  *stream : Pointer to the DMA stream
 * @param   incr   : Valid parameters are [INCR_ENABLE, INCR_DISABLE]
 * @return DMAC_RETURN_CODE_t
 *
 * <br>
 * <b>Affected register and bit(s)</b><br>
 * <table>
 *      <tr>
 *          <th>Register</th>
 *          <th>Bit name</th>
 *          <th>Bit(s)</th>
 *      </tr>
 *      <tr>
 *          <td>SxCR</td>
 *          <td rowspan="1">PINC</td>
 *          <td rowspan="1">9</td>
 *      </tr>
 * </table>
 */
DMAC_RETURN_CODE_t dmacSetPeripheralIncrementMode(DMA_Stream_TypeDef *stream, DMAC_INCR_t incr)
{
    if (dmacVerifyIncrementMode(incr) != true)
    {
        return DMAC_INVALID_INCREMENTAL_MODE;
    }

    stream->CR &= ~DMA_SxCR_PINC_Msk;   // Reset previous setting
    switch (incr)
    {
        case INCR_ENABLE:
            stream->CR |= DMA_SxCR_PINC;
            break;
        case INCR_DISABLE:
            stream->CR &= ~DMA_SxCR_PINC_Msk;
            break;
        default:
            stream->CR &= ~DMA_SxCR_PINC_Msk;
            break;
    }
    return DMAC_OK;
}

/**
 * @ingroup dma2
 * @brief Configures the memory burst transfer
 *
 * Burst transfer is typically used when a sequence of data has to be transmitted. By default, each single data (byte, half<br>
 * word, or word) is immediately transferred via the AHB. By configuring the burst transfer it is possible to "collect" a set<br>
 * of multiple data which are then transferred at once. This decreases the load of the AHB.<br>
 *
 * @param   *stream : Pointer to the DMA stream
 * @param    burst  : Number of DMA burst beats
 * @return  DMAC_RETURN_CODE_t
 *
 * @note
 * Don't mix up the burst size with the number of data configured by using the NDTR register. The reference manual mentions<br>
 * so-called <b>beats</b>. One beat is not necessarily a byte: It can also be a half word or a word!
 *
 * <br>
 * <b>Affected register and bit(s)</b><br>
 * <table>
 *      <tr>
 *          <th>Register</th>
 *          <th>Bit name</th>
 *          <th>Bit(s)</th>
 *      </tr>
 *      <tr>
 *          <td>SxCR</td>
 *          <td rowspan="1">MBURST</td>
 *          <td rowspan="1">24...23</td>
 *      </tr>
 * </table>
 */
DMAC_RETURN_CODE_t dmacSetMemoryBurstBeats(DMA_Stream_TypeDef *stream, DMAC_NUM_BEATS_t burst)
{
    if (dmacVerifyNumBeats(burst) != true)
    {
        return DMAC_INVALID_NUM_BEATS;
    }

    stream->CR &= ~DMA_SxCR_MBURST_Msk;
    switch (burst)
    {
        case BEATSIZE_1:
            stream->CR &= ~DMA_SxCR_MBURST_Msk;
            break;
        case BEATSIZE_4:
            stream->CR |= DMA_SxCR_MBURST_0;
            break;
        case BEATSIZE_8:
            stream->CR |= DMA_SxCR_MBURST_1;
            break;
        case BEATSIZE_16:
            stream->CR |= (DMA_SxCR_MBURST_1 | DMA_SxCR_MBURST_0);
            break;
        default:
            stream->CR |= ~DMA_SxCR_MBURST_Msk;
            break;

    }
    return DMAC_OK;
}

/**
 * @ingroup dma2
 * @brief Configures the peripheral burst transfer
 *
 * Burst transfer is typically used when a sequence of data has to be transmitted. By default, each single data (byte, half<br>
 * word, or word) is immediately transferred via the AHB. By configuring the burst transfer it is possible to "collect" a set<br>
 * of multiple data which are then transferred at once. This decreases the load of the AHB.
 *
 * @param   *stream : Pointer to the DMA stream
 * @param    burst  : Number of DMA burst beats
 * @return  DMAC_RETURN_CODE_t
 *
 * @note
 * Don't mix up the burst size with the number of data configured by using the NDTR register. The reference manual mentions<br>
 * so-called <b>beats</b>. One beat is not necessarily a byte: It can also be a half word or a word!
 *
 * <br>
 * <b>Affected register and bit(s)</b><br>
 * <table>
 *      <tr>
 *          <th>Register</th>
 *          <th>Bit name</th>
 *          <th>Bit(s)</th>
 *      </tr>
 *      <tr>
 *          <td>SxCR</td>
 *          <td rowspan="1">PBURST</td>
 *          <td rowspan="1">22...21</td>
 *      </tr>
 * </table>
 */
DMAC_RETURN_CODE_t dmacSetPeripheralBurstBeats(DMA_Stream_TypeDef *stream, DMAC_NUM_BEATS_t burst)
{
    if (dmacVerifyNumBeats(burst) != true)
    {
        return DMAC_INVALID_NUM_BEATS;
    }

    stream->CR &= ~DMA_SxCR_PBURST_Msk;
    switch (burst)
    {
        case BEATSIZE_1:
            stream->CR &= ~DMA_SxCR_PBURST_Msk;
            break;
        case BEATSIZE_4:
            stream->CR |= DMA_SxCR_PBURST_0;
            break;
        case BEATSIZE_8:
            stream->CR |= DMA_SxCR_PBURST_1;
            break;
        case BEATSIZE_16:
            stream->CR |= (DMA_SxCR_PBURST_1 | DMA_SxCR_PBURST_0);
            break;
        default:
            stream->CR |= ~DMA_SxCR_PBURST_Msk;
            break;

    }
    return DMAC_OK;
}

/**
 * @ingroup dma2
 * @brief Configures the priority level of a DMA stream
 *
 * The arbiter uses four priority levels to decide which stream must be handled first. These levels are LOW, MEDIUM, HIGH, and<br>
 * VERY HIGH. If two streams are configured to have the same priority, the stream with the lower number takes priority over the<br>
 * stream with the higher number.
 *
 * @param   *stream : Pointer to the DMA stream
 * @param    prio   : Defines the DMA priority level
 * @return  DMAC_RETURN_CODE_t
 *
 * <br>
 * <b>Affected register and bit(s)</b><br>
 * <table>
 *      <tr>
 *          <th>Register</th>
 *          <th>Bit name</th>
 *          <th>Bit(s)</th>
 *      </tr>
 *      <tr>
 *          <td>SxCR</td>
 *          <td rowspan="1">PL</td>
 *          <td rowspan="1">17...16</td>
 *      </tr>
 * </table>
 */
DMAC_RETURN_CODE_t dmacSetPriorityLevel(DMA_Stream_TypeDef *stream, DMAC_PRIORITY_LEVEL_t prio)
{
    if (dmacVerifyPriorityLevel(prio) != true)
    {
        return DMAC_INVALID_PRIORITY_SETTING;
    }

    stream->CR &= ~DMA_SxCR_PL_Msk;     // Reset priority level before setting a new priority
    switch (prio)
    {
        case PRIO_LOW:
            stream->CR &= ~DMA_SxCR_PL_Msk;
            break;
        case PRIO_MEDIUM:
            stream->CR |= DMA_SxCR_PL_0;
            break;
        case PRIO_HIGH:
            stream->CR |= DMA_SxCR_PL_1;
            break;
        case PRIO_VERY_HIGH:
            stream->CR |= (DMA_SxCR_PL_0 | DMA_SxCR_PL_1);
            break;
        default:
            stream->CR &= ~DMA_SxCR_PL_Msk;
            break;
    }
    return DMAC_OK;
}

/**
 * @ingroup dma3
 * Enables the desired interrupt type. This function is also used by <b>dmacEnableInterruptList(...)</b>.
 *
 * @param   *stream  : Pointer to the DMA stream
 * @param    irqType : Single interrupt which shall be enabled
 * @return  DMAC_RETURN_CODE_t
 *
 * <br>
 * <b>Affected register and bit(s)</b><br>
 * <table>
 *      <tr>
 *          <th>Register</th>
 *          <th>Bit name</th>
 *          <th>Bit(s)</th>
 *      </tr>
 *      <tr>
 *          <td>SxCR</td>
 *          <td rowspan="1">Different interrupts</td>
 *          <td rowspan="1">4...1</td>
 *      </tr>
 * </table>
 */
DMAC_RETURN_CODE_t dmacEnableInterrupt(DMA_Stream_TypeDef *stream, DMAC_IRQ_t irqType)
{
    if (dmacVerifyIrqType(irqType) == false)
    {
        return DMAC_INVALID_IRQ_TYPE;
    }

    switch (irqType)
    {
        case DIRECT_MODE_ERR:
            stream->CR |= DMA_SxCR_DMEIE;
            break;

        case TX_ERR:
            stream->CR |= DMA_SxCR_TEIE;
            break;

        case TX_HALF:
            stream->CR |= DMA_SxCR_HTIE;
            break;

        case TX_COMPLETE:
            stream->CR |= DMA_SxCR_TCIE;
            break;

        case FIFO_ERROR_ENABLE:
            stream->FCR |= DMA_SxFCR_FEIE;
            break;
    }
    return DMAC_OK;
}

/**
 * @ingroup dma3
 * Enables the interrupts which are listed in the irqList array.
 *
 * @param   *stream  : Pointer to the DMA stream
 * @param   *irqList : Pointer to the array which provides the interrupt types (Usage: DMAC_IRQ_t[] = {IRQ1, IRQ2, ..., IRQn};)
 * @param    size    : size of the irqList array
 * @return  DMAC_RETURN_CODE_t
 *
 * <br>
 * <b>Affected register and bit(s)</b><br>
 * <table>
 *      <tr>
 *          <th>Register</th>
 *          <th>Bit name</th>
 *          <th>Bit(s)</th>
 *      </tr>
 *      <tr>
 *          <td>SxCR</td>
 *          <td rowspan="1">Different interrupts</td>
 *          <td rowspan="1">4...1</td>
 *      </tr>
 * </table>
 */
DMAC_RETURN_CODE_t dmacEnableInterruptList(DMA_Stream_TypeDef *stream, DMAC_IRQ_t *irqList, uint8_t size)
{
    uint8_t    i = 0;
    DMAC_IRQ_t irqType;

    for (i = 0; i < size; i++)
    {
        irqType = irqList[i];
        dmacEnableInterrupt(stream, irqType);
    }

    return DMAC_OK;
}

/**
 * @ingroup dma3
 * Disables the desired interrupt type. This function is also used by <b>dmacDisableInterruptList(...)</b>.
 *
 * @param   *stream  : Pointer to the DMA stream
 * @param    irqType : Single interrupt which shall be enabled
 * @return  DMAC_RETURN_CODE_t
 *
 * <br>
 * <b>Affected register and bit(s)</b><br>
 * <table>
 *      <tr>
 *          <th>Register</th>
 *          <th>Bit name</th>
 *          <th>Bit(s)</th>
 *      </tr>
 *      <tr>
 *          <td>SxCR</td>
 *          <td rowspan="1">Different interrupts</td>
 *          <td rowspan="1">4...1</td>
 *      </tr>
 * </table>
 */
DMAC_RETURN_CODE_t dmacDisableInterrupt(DMA_Stream_TypeDef *stream, DMAC_IRQ_t irqType)
{
    if (dmacVerifyIrqType(irqType) == false)
    {
        return DMAC_INVALID_IRQ_TYPE;
    }

    switch (irqType)
    {
        case DIRECT_MODE_ERR:
            stream->CR &= ~DMA_SxCR_DMEIE_Msk;
            break;

        case TX_ERR:
            stream->CR &= ~DMA_SxCR_TEIE_Msk;
            break;

        case TX_HALF:
            stream->CR &= ~DMA_SxCR_HTIE_Msk;
            break;

        case TX_COMPLETE:
            stream->CR &= ~DMA_SxCR_TCIE_Msk;
            break;

        case FIFO_ERROR_ENABLE:
            stream->FCR &= ~DMA_SxFCR_FEIE_Msk;
            break;
    }
    return DMAC_OK;
}

/**
 * @ingroup dma3
 * Disables the interrupts which are listed in the irqList array.
 *
 * @param   *stream  : Pointer to the DMA stream
 * @param   *irqList : Pointer to the array which provides the interrupt types (Usage: DMAC_IRQ_t[] = {IRQ1, IRQ2, ..., IRQn};)
 * @param    size    : size of the irqList array
 * @return  DMAC_RETURN_CODE_t
 *
 * <br>
 * <b>Affected register and bit(s)</b><br>
 * <table>
 *      <tr>
 *          <th>Register</th>
 *          <th>Bit name</th>
 *          <th>Bit(s)</th>
 *      </tr>
 *      <tr>
 *          <td>SxCR</td>
 *          <td rowspan="1">Different interrupts</td>
 *          <td rowspan="1">4...1</td>
 *      </tr>
 * </table>
 */
DMAC_RETURN_CODE_t dmacDisableInterruptList(DMA_Stream_TypeDef *stream, DMAC_IRQ_t *irqList, uint8_t size)
{
    uint8_t    i = 0;
    DMAC_IRQ_t irqType;

    for (i = 0; i < size; i++)
    {
        irqType = irqList[i];
        dmacDisableInterrupt(stream, irqType);
    }

    return DMAC_OK;
}

/**
 * @ingroup dma3
 * Returns the content of the DMA interrupt status register for DMA streams 0 to 3.
 *
 * @param   *dma    : Pointer to the DMA controller
 * @return  Content of the LISR register
 *
 * <br>
 * <b>Affected register and bit(s)</b><br>
 * <table>
 *      <tr>
 *          <th>Register</th>
 *          <th>Bit name</th>
 *          <th>Bit(s)</th>
 *      </tr>
 *      <tr>
 *          <td>LISR</td>
 *          <td rowspan="1">Different interrupts</td>
 *          <td rowspan="1"></td>
 *      </tr>
 * </table>
 */
uint32_t dmacGetLowInterruptStatus(DMA_TypeDef *dma)
{
    return dma->LISR;
}

/**
 * @ingroup dma3
 * Returns the content of the DMA interrupt status register for DMA streams 4 to 7.
 *
 * @param   *dma  : Pointer to the DMA controller
 * @return  Content of the HISR register
 *
 * <br>
 * <b>Affected register and bit(s)</b><br>
 * <table>
 *      <tr>
 *          <th>Register</th>
 *          <th>Bit name</th>
 *          <th>Bit(s)</th>
 *      </tr>
 *      <tr>
 *          <td>HISR</td>
 *          <td rowspan="1">Different interrupts</td>
 *          <td rowspan="1"></td>
 *      </tr>
 * </table>
 */
uint32_t dmacGetHighInterruptStatus(DMA_TypeDef *dma)
{
    return dma->HISR;
}

/**
 * @ingroup dma2
 * Selects whether the DMAC or the peripheral shall be the DMA flow controller.
 *
 * @param   *stream   : Pointer to the DMA stream
 * @param    flowCtrl : Might be either DMA_IS_CTRL or PERIPHERAL_IS_CTRL
 * @return  DMAC_RETURN_CODE_t
 *
 * <br>
 * <b>Affected register and bit(s)</b><br>
 * <table>
 *      <tr>
 *          <th>Register</th>
 *          <th>Bit name</th>
 *          <th>Bit(s)</th>
 *      </tr>
 *      <tr>
 *          <td>SxCR</td>
 *          <td rowspan="1">PFCTRL</td>
 *          <td rowspan="1">5</td>
 *      </tr>
 * </table>
 */
DMAC_RETURN_CODE_t dmacSelectDMAFlowController(DMA_Stream_TypeDef *stream, DMAC_DMA_FLOW_CONTROLLER_t flowCtrl)
{
    if (dmacVerifyDMAFlowController(flowCtrl) == false)
    {
        return DMAC_INVALID_FLOW_CONTROLLER;
    }

    switch (flowCtrl)
    {
        case DMA_IS_CTRL:
            stream->CR &= ~DMA_SxCR_PFCTRL_Msk;
            break;

        case PERIPHERAL_IS_CTRL:
            stream->CR |= DMA_SxCR_PFCTRL;
    }

    return DMAC_OK;
}

/**
 * @ingroup dma4
 * Enables the DMA circular mode.
 *
 * @param   *stream : Pointer to the DMA stream
 * @return  DMAC_RETURN_CODE_t
 *
 * <br>
 * <b>Affected register and bit(s)</b><br>
 * <table>
 *      <tr>
 *          <th>Register</th>
 *          <th>Bit name</th>
 *          <th>Bit(s)</th>
 *      </tr>
 *      <tr>
 *          <td>SxCR</td>
 *          <td rowspan="1">CIRC</td>
 *          <td rowspan="1">8</td>
 *      </tr>
 * </table>
 */
DMAC_RETURN_CODE_t dmacEnableCircularMode(DMA_Stream_TypeDef *stream)
{
    stream->CR |= DMA_SxCR_CIRC;
    return DMAC_OK;
}

/**
 * @ingroup dma4
 * Disables the DMA circular mode.
 *
 * @param   *stream : Pointer to the DMA stream
 * @return  DMAC_RETURN_CODE_t
 *
 * <br>
 * <b>Affected register and bit(s)</b><br>
 * <table>
 *      <tr>
 *          <th>Register</th>
 *          <th>Bit name</th>
 *          <th>Bit(s)</th>
 *      </tr>
 *      <tr>
 *          <td>SxCR</td>
 *          <td rowspan="1">CIRC</td>
 *          <td rowspan="1">8</td>
 *      </tr>
 * </table>
 */
DMAC_RETURN_CODE_t dmacDisableCircularMode(DMA_Stream_TypeDef *stream)
{
    stream->CR &= ~DMA_SxCR_CIRC_Msk;
    return DMAC_OK;
}

/**
 * @ingroup dma2
 * Sets the address offset of the peripheral.
 *
 * @param   *stream : Pointer to the DMA stream
 * @param    offset : Sets the offset either to the same value as PSIZE or to 32-bit alignment (word)
 * @return  DMAC_RETURN_CODE_t
 *
 * <br>
 * <b>Affected register and bit(s)</b><br>
 * <table>
 *      <tr>
 *          <th>Register</th>
 *          <th>Bit name</th>
 *          <th>Bit(s)</th>
 *      </tr>
 *      <tr>
 *          <td>SxCR</td>
 *          <td rowspan="1">PINCOS</td>
 *          <td rowspan="1">15</td>
 *      </tr>
 * </table>
 */
DMAC_RETURN_CODE_t dmacConfigPeriphOffsSize(DMA_Stream_TypeDef *stream, DMAC_PERIPHERAL_INCR_OFFS_SIZE_t offset)
{
    if (dmacVerifyPeripheralOffsetSize(offset) == false)
    {
        return DMAC_INVALID_OFFSET_SIZE;
    }

    switch (offset)
    {
        case PSIZE_VAL:
            stream->CR &= ~DMA_SxCR_PINCOS_Msk;
            break;

        case FIXED_TO_WORD:
            stream->CR |= DMA_SxCR_PINCOS;
            break;
    }
    return DMAC_OK;
}

/**
 * @ingroup dma4
 * Enables the DMA double-buffer mode
 *
 * @param   *stream : Pointer to the DMA stream
 * @return  DMAC_RETURN_CODE_t
 *
 * <br>
 * <b>Affected register and bit(s)</b><br>
 * <table>
 *      <tr>
 *          <th>Register</th>
 *          <th>Bit name</th>
 *          <th>Bit(s)</th>
 *      </tr>
 *      <tr>
 *          <td>SxCR</td>
 *          <td rowspan="1">DBM</td>
 *          <td rowspan="1">18</td>
 *      </tr>
 * </table>
 */
DMAC_RETURN_CODE_t dmacEnableDoubleBufferMode(DMA_Stream_TypeDef *stream)
{
    stream->CR |= DMA_SxCR_DBM;
    return DMAC_OK;
}

/**
 * @ingroup dma4
 * Disables the DMA double-buffer mode
 *
 * @param   *stream : Pointer to the DMA stream
 * @return  DMAC_RETURN_CODE_t
 *
 * <br>
 * <b>Affected register and bit(s)</b><br>
 * <table>
 *      <tr>
 *          <th>Register</th>
 *          <th>Bit name</th>
 *          <th>Bit(s)</th>
 *      </tr>
 *      <tr>
 *          <td>SxCR</td>
 *          <td rowspan="1">DBM</td>
 *          <td rowspan="1">18</td>
 *      </tr>
 * </table>
 */
DMAC_RETURN_CODE_t dmacDisableDoubleBufferMode(DMA_Stream_TypeDef *stream)
{
    stream->CR &= ~DMA_SxCR_DBM_Msk;
    return DMAC_OK;
}

/**
 * @ingroup dma4
 * Selects the target memory (current target).
 *
 * @param   *stream : Pointer to the DMA stream
 * @param    mem    : Might be MEM_0 or MEM_1
 * @return  DMAC_RETURN_CODE_t
 *
 * @note
 * This function can only be used when the DMAC is configured for double-buffer mode.
 *
 * <br>
 * <b>Affected register and bit(s)</b><br>
 * <table>
 *      <tr>
 *          <th>Register</th>
 *          <th>Bit name</th>
 *          <th>Bit(s)</th>
 *      </tr>
 *      <tr>
 *          <td>SxCR</td>
 *          <td rowspan="1">CT</td>
 *          <td rowspan="1">19</td>
 *      </tr>
 * </table>
 */
DMAC_RETURN_CODE_t dmacSelectMem2MemTarget(DMA_Stream_TypeDef *stream, DMAC_CURRENT_TARGET_MEM_t mem)
{
    if (!(stream->CR & DMA_SxCR_DBM))
    {
        return DMAC_NOT_IN_DOUBLE_BUFFER_MODE;
    }

    if (dmacVerifyCurrentTargetMem(mem) == false)
    {
        return DMAC_INVALID_TARGET_MEMORY;
    }
    switch (mem)
    {
        case CT_IS_MEM_0:
            stream->CR &= ~DMA_SxCR_CT_Msk;
            break;

        case CT_IS_MEM_1:
            stream->CR |= DMA_SxCR_CT;
            break;
    }

    return DMAC_OK;
}

/**
 * @ingroup dma4
 * Enables the DMA FIFO direct mode.
 *
 * @param   *stream : Pointer to the DMA stream
 * @return  DMAC_RETURN_CODE_t
 *
 * @note
 * Consider that the DMDIS bit must be '0' to enable the direct mode.
 *
 * <br>
 * <b>Affected register and bit(s)</b><br>
 * <table>
 *      <tr>
 *          <th>Register</th>
 *          <th>Bit name</th>
 *          <th>Bit(s)</th>
 *      </tr>
 *      <tr>
 *          <td>FCR</td>
 *          <td rowspan="1">DMDIS</td>
 *          <td rowspan="1">2</td>
 *      </tr>
 * </table>
 */
DMAC_RETURN_CODE_t dmacEnableFifoMode(DMA_Stream_TypeDef *stream)
{
    stream->FCR &= ~DMA_SxFCR_DMDIS_Msk;
    return DMAC_OK;
}

/**
 * @ingroup dma4
 * Disables the DMA FIFO direct mode.
 *
 * @param   *stream : Pointer to the DMA stream
 * @return  DMAC_RETURN_CODE_t
 *
 * @note
 * Consider that the DMDIS bit must be '1' to disable the direct mode. Direct mode is forbidden if the data flow-direction is<br>
 * MEM_2_MEM.
 *
 * @note
 * Disabling the DMA FIFO mode implicetely enables the 'Direct mode' (DMDIS bit in FCR is then reset to 0).
 *
 * <br>
 * <b>Affected register and bit(s)</b><br>
 * <table>
 *      <tr>
 *          <th>Register</th>
 *          <th>Bit name</th>
 *          <th>Bit(s)</th>
 *      </tr>
 *      <tr>
 *          <td>FCR</td>
 *          <td rowspan="1">DMDIS</td>
 *          <td rowspan="1">2</td>
 *      </tr>
 * </table>
 */
DMAC_RETURN_CODE_t dmacDisableFifoMode(DMA_Stream_TypeDef *stream)
{
    stream->FCR |= DMA_SxFCR_DMDIS;
    return DMAC_OK;
}

/**
 * @ingroup dma4
 * Sets the FIFO threshold level.
 *
 * @param   *stream : Pointer to the DMA stream
 * @param    fth    : Threshold level
 * @return  DMAC_RETURN_CODE_t
  *
 * <br>
 * <b>Affected register and bit(s)</b><br>
 * <table>
 *      <tr>
 *          <th>Register</th>
 *          <th>Bit name</th>
 *          <th>Bit(s)</th>
 *      </tr>
 *      <tr>
 *          <td>FCR</td>
 *          <td rowspan="1">FTH</td>
 *          <td rowspan="1">1...0</td>
 *      </tr>
 * </table>
 */
DMAC_RETURN_CODE_t dmacSetFifoThreshold(DMA_Stream_TypeDef *stream, DMAC_FIFO_THRESHOLD_t fth)
{
    if (dmacVerifyFifoThresholdLvl(fth) == false)
    {
        return DMAC_INVALID_THRESHOLD_LEVEL;
    }

    // Reset previous settings before setting the new threshold level
    stream->FCR &= ~DMA_SxFCR_FTH_Msk;

    switch (fth)
    {
        case ONE_FOURTH:
            stream->FCR &= ~DMA_SxFCR_FTH_Msk;
            break;

        case HALF:
            stream->FCR |= DMA_SxFCR_FTH_0;
            break;

        case THREE_FOURTH:
            stream->FCR |= DMA_SxFCR_FTH_1;
            break;

        case FULL:
            stream->FCR |= DMA_SxFCR_FTH_1 | DMA_SxFCR_FTH_0;
            break;
    }
    return DMAC_OK;
}

