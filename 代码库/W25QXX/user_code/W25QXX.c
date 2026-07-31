#include "W25QXX.h"

#include <string.h>

#define W25QXX_CMD_WRITE_ENABLE      0x06U
#define W25QXX_CMD_WRITE_STATUS      0x01U
#define W25QXX_CMD_READ_STATUS1      0x05U
#define W25QXX_CMD_READ_DATA         0x03U
#define W25QXX_CMD_PAGE_PROGRAM      0x02U
#define W25QXX_CMD_SECTOR_ERASE      0x20U
#define W25QXX_CMD_JEDEC_ID          0x9FU

#define W25QXX_STATUS1_BUSY_MASK     0x01U
#define W25QXX_STATUS1_WEL_MASK      0x02U
#define W25QXX_COMMAND_SIZE           4U
#define W25QXX_JEDEC_ID_SIZE          3U
#define W25QXX_TRANSFER_CHUNK_SIZE    W25QXX_PAGE_SIZE
#define W25QXX_PAGE_PROGRAM_TIMEOUT_MS 10U
#define W25QXX_STATUS_WRITE_TIMEOUT_MS 100U
#define W25QXX_SECTOR_ERASE_TIMEOUT_MS 1000U
#define W25QXX_POWER_UP_WRITE_DELAY_MS  5U

/**
 * @brief 使用 DMA 完成一次全双工 SPI 数据交换。
 * @note 上层负责控制 CS；发生超时时会中止 DMA，使 SPI 恢复到可用状态。
 */
static bool W25QXX_Transfer(w25qxx_t *flash, const uint8_t *tx,
                             uint8_t *rx, uint16_t length)
{
    uint32_t start_tick;

    if ((flash == NULL) || (flash->spi == NULL))
    {
        return false;
    }

    if (!BspSpi_TransferDma(flash->spi, tx, rx, length))
    {
        return false;
    }

    start_tick = HAL_GetTick();
    while (!BspSpi_IsDmaReady(flash->spi))
    {
        if ((uint32_t)(HAL_GetTick() - start_tick)
            >= BSP_SPI_TRANSFER_TIMEOUT_MS)
        {
            BspSpi_AbortDma(flash->spi);
            return false;
        }
    }

    return BspSpi_GetError(flash->spi) == HAL_SPI_ERROR_NONE;
}

/** @brief 生成包含操作码和 24 位地址的四字节命令。 */
static void W25QXX_BuildAddressCommand(uint8_t *command, uint8_t opcode,
                                       uint32_t address)
{
    command[0] = opcode;
    command[1] = (uint8_t)(address >> 16U);
    command[2] = (uint8_t)(address >> 8U);
    command[3] = (uint8_t)address;
}

/** @brief 读取状态寄存器 1，用于查询 BUSY 和 WEL 位。 */
static bool W25QXX_ReadStatus1(w25qxx_t *flash, uint8_t *status1)
{
    uint8_t tx[2] = {W25QXX_CMD_READ_STATUS1, 0U};
    uint8_t rx[2];
    bool result;

    if ((flash == NULL) || (status1 == NULL))
    {
        return false;
    }

    BspSpi_CS_Low(flash->spi);
    result = W25QXX_Transfer(flash, tx, rx, sizeof(tx));
    BspSpi_CS_High(flash->spi);
    if (!result)
    {
        return false;
    }

    *status1 = rx[1];
    return true;
}

/** @brief 发送写使能命令，并确认 WEL 位已经置位。 */
static bool W25QXX_EnableWrite(w25qxx_t *flash)
{
    uint8_t tx = W25QXX_CMD_WRITE_ENABLE;
    uint8_t rx;
    uint8_t status1;
    bool result;

    if ((flash == NULL) || (flash->spi == NULL))
    {
        return false;
    }

    BspSpi_CS_Low(flash->spi);
    result = W25QXX_Transfer(flash, &tx, &rx, 1U);
    BspSpi_CS_High(flash->spi);

    /* WREN 结束后保持 CS 为高，确保芯片锁存写使能命令。 */
    /* WEL=1 表示芯片已接受写使能指令；否则擦除和写页会被忽略。 */
    return result && W25QXX_ReadStatus1(flash, &status1)
           && ((status1 & W25QXX_STATUS1_WEL_MASK) != 0U);
}

/** @brief 检查访问范围是否位于当前 Flash 容量以内。 */
static bool W25QXX_IsRangeValid(const w25qxx_t *flash, uint32_t address,
                                 uint32_t length)
{
    if ((flash == NULL) || !flash->initialized || (length == 0U)
        || (address >= flash->capacity_bytes))
    {
        return false;
    }

    return length <= (flash->capacity_bytes - address);
}

/**
 * @brief 写入一个页内的数据，调用者必须保证数据不跨越 256 字节页边界。
 */
static bool W25QXX_PageProgram(w25qxx_t *flash, uint32_t address,
                               const uint8_t *data, uint16_t length)
{
    uint8_t command[W25QXX_COMMAND_SIZE];
    uint8_t discard[W25QXX_PAGE_SIZE];
    bool result;

    if ((data == NULL) || (length == 0U) || (length > W25QXX_PAGE_SIZE)
        || ((address & (W25QXX_PAGE_SIZE - 1U)) + length > W25QXX_PAGE_SIZE))
    {
        return false;
    }

    if (!W25QXX_WaitReady(flash, W25QXX_PAGE_PROGRAM_TIMEOUT_MS)
        || !W25QXX_EnableWrite(flash))
    {
        return false;
    }

    W25QXX_BuildAddressCommand(command, W25QXX_CMD_PAGE_PROGRAM, address);
    BspSpi_CS_Low(flash->spi);
    result = W25QXX_Transfer(flash, command, discard, sizeof(command));
    if (result)
    {
        result = W25QXX_Transfer(flash, data, discard, length);
    }
    BspSpi_CS_High(flash->spi);
    return result;
}

/** @brief 初始化驱动，读取 JEDEC ID 并确定芯片型号和容量。 */
bool W25QXX_Init(w25qxx_t *flash, bsp_spi_t *spi)
{
    uint32_t jedec_id;

    if ((flash == NULL) || (spi == NULL) || !spi->initialized)
    {
        return false;
    }

    flash->spi = spi;
    flash->jedec_id = 0U;
    flash->capacity_bytes = 0U;
    flash->model = W25QXX_MODEL_UNKNOWN;
    flash->initialized = false;

    HAL_Delay(W25QXX_POWER_UP_WRITE_DELAY_MS);

    if (!W25QXX_ReadJedecId(flash, &jedec_id))
    {
        return false;
    }

    flash->jedec_id = jedec_id;
    if (jedec_id == W25QXX_JEDEC_ID_W25Q64)
    {
        flash->model = W25QXX_MODEL_W25Q64;
        flash->capacity_bytes = W25Q64_CAPACITY_BYTES;
    }
    else if (jedec_id == W25QXX_JEDEC_ID_W25Q128)
    {
        flash->model = W25QXX_MODEL_W25Q128;
        flash->capacity_bytes = W25Q128_CAPACITY_BYTES;
    }
    else
    {
        return false;
    }

    flash->initialized = true;
    return true;
}

/** @brief 读取三字节 JEDEC ID。 */
bool W25QXX_ReadJedecId(w25qxx_t *flash, uint32_t *jedec_id)
{
    uint8_t tx[W25QXX_JEDEC_ID_SIZE + 1U] = {W25QXX_CMD_JEDEC_ID, 0U, 0U, 0U};
    uint8_t rx[W25QXX_JEDEC_ID_SIZE + 1U];
    bool result;

    if ((flash == NULL) || (flash->spi == NULL) || (jedec_id == NULL))
    {
        return false;
    }

    BspSpi_CS_Low(flash->spi);
    result = W25QXX_Transfer(flash, tx, rx, sizeof(tx));
    BspSpi_CS_High(flash->spi);
    if (!result)
    {
        return false;
    }

    *jedec_id = ((uint32_t)rx[1] << 16U) | ((uint32_t)rx[2] << 8U) | rx[3];
    return true;
}

/** @brief 返回 Flash 容量，单位为字节。 */
uint32_t W25QXX_GetCapacity(const w25qxx_t *flash)
{
    return (flash != NULL) ? flash->capacity_bytes : 0U;
}

/** @brief 轮询 BUSY 位，等待芯片内部编程或擦除操作结束。 */
bool W25QXX_WaitReady(w25qxx_t *flash, uint32_t timeout_ms)
{
    uint8_t status1;
    uint32_t start_tick;

    if ((flash == NULL) || (flash->spi == NULL))
    {
        return false;
    }

    start_tick = HAL_GetTick();
    do
    {
        if (!W25QXX_ReadStatus1(flash, &status1))
        {
            return false;
        }
        if ((status1 & W25QXX_STATUS1_BUSY_MASK) == 0U)
        {
            return true;
        }
    } while ((HAL_GetTick() - start_tick) < timeout_ms);

    return false;
}

/** @brief 清除状态寄存器中的块保护配置。 */
bool W25QXX_Unprotect(w25qxx_t *flash)
{
    uint8_t command[3] = {W25QXX_CMD_WRITE_STATUS, 0U, 0U};
    uint8_t discard[sizeof(command)];
    bool result;

    if ((flash == NULL) || !flash->initialized
        || !W25QXX_WaitReady(flash, W25QXX_STATUS_WRITE_TIMEOUT_MS)
        || !W25QXX_EnableWrite(flash))
    {
        return false;
    }

    /*
     * SR1=0 清除 BP/TB/SEC 块保护位，SR2=0 清除 CMP 反向保护位。
     * 本工程使用普通 SPI，不需要保留 SR2 中的 QE 位。
     */
    BspSpi_CS_Low(flash->spi);
    result = W25QXX_Transfer(flash, command, discard, sizeof(command));
    BspSpi_CS_High(flash->spi);

    return result && W25QXX_WaitReady(flash, W25QXX_STATUS_WRITE_TIMEOUT_MS);
}

/**
 * @brief 从指定地址连续读取数据。
 * @note 命令阶段和数据阶段之间 CS 始终保持低电平。
 */
bool W25QXX_Read(w25qxx_t *flash, uint32_t address, uint8_t *data,
                  uint32_t length)
{
    uint8_t command[W25QXX_COMMAND_SIZE];
    uint8_t discard[W25QXX_COMMAND_SIZE];
    uint8_t dummy[W25QXX_TRANSFER_CHUNK_SIZE];
    uint16_t chunk_size;
    bool result;

    if ((data == NULL) || !W25QXX_IsRangeValid(flash, address, length))
    {
        return false;
    }

    W25QXX_BuildAddressCommand(command, W25QXX_CMD_READ_DATA, address);
    memset(dummy, 0xFF, sizeof(dummy));
    BspSpi_CS_Low(flash->spi);
    result = W25QXX_Transfer(flash, command, discard, sizeof(command));
    while (result && (length > 0U))
    {
        chunk_size = (length > W25QXX_TRANSFER_CHUNK_SIZE)
                   ? W25QXX_TRANSFER_CHUNK_SIZE : (uint16_t)length;
        result = W25QXX_Transfer(flash, dummy, data, chunk_size);
        data += chunk_size;
        length -= chunk_size;
    }
    BspSpi_CS_High(flash->spi);
    return result;
}

/** @brief 连续写入数据，函数会自动按照页边界拆分。 */
bool W25QXX_Write(w25qxx_t *flash, uint32_t address, const uint8_t *data,
                   uint32_t length)
{
    uint32_t page_remaining;
    uint16_t chunk_size;

    if ((data == NULL) || !W25QXX_IsRangeValid(flash, address, length))
    {
        return false;
    }

    while (length > 0U)
    {
        page_remaining = W25QXX_PAGE_SIZE - (address & (W25QXX_PAGE_SIZE - 1U));
        chunk_size = (length < page_remaining) ? (uint16_t)length
                                                 : (uint16_t)page_remaining;
        if (!W25QXX_PageProgram(flash, address, data, chunk_size)
            || !W25QXX_WaitReady(flash, BSP_SPI_TRANSFER_TIMEOUT_MS))
        {
            return false;
        }

        address += chunk_size;
        data += chunk_size;
        length -= chunk_size;
    }

    return true;
}

/** @brief 擦除地址所在的 4 KB 扇区。 */
bool W25QXX_EraseSector(w25qxx_t *flash, uint32_t address)
{
    uint8_t command[W25QXX_COMMAND_SIZE];
    uint8_t discard[W25QXX_COMMAND_SIZE];
    bool result;

    if (!W25QXX_IsRangeValid(flash, address, 1U)
        || !W25QXX_WaitReady(flash, W25QXX_SECTOR_ERASE_TIMEOUT_MS)
        || !W25QXX_EnableWrite(flash))
    {
        return false;
    }

    address &= ~(W25QXX_SECTOR_SIZE - 1U);
    W25QXX_BuildAddressCommand(command, W25QXX_CMD_SECTOR_ERASE, address);
    BspSpi_CS_Low(flash->spi);
    result = W25QXX_Transfer(flash, command, discard, sizeof(command));
    BspSpi_CS_High(flash->spi);
    return result && W25QXX_WaitReady(flash, W25QXX_SECTOR_ERASE_TIMEOUT_MS);
}
