/**
 * @file    cali_store.c
 * @brief   IMU 标定数据的 Flash 掉电保存
 *
 * 用 magic + version + length + CRC16 四重校验判断数据是否可信。
 * 空白 Flash 全是 0xFF，magic 一定对不上，所以第一次上电会自然走"无有效数据"分支。
 *
 * 注意：擦除一页约 20~40ms，这期间 Flash 接口被占用，CPU 连取指都会被卡住，
 * 中断也跑不了。所以只能在电机停转时调用 CaliStore_Save()/Erase()——
 * 平衡控制期间擦 Flash，小车必倒。
 */

#include "cali_store.h"
#include "crc16.h"
#include <stddef.h>

/* Datas ---------------------------------------------------------------------*/
static CaliData_tTypeDef caliData;
static uint8_t           caliValid = 0U;

/* Flash 按半字(16bit)编程，结构体长度必须是偶数 */
_Static_assert((sizeof(CaliData_tTypeDef) % 2U) == 0U, "CaliData_tTypeDef size must be even");
_Static_assert(sizeof(CaliData_tTypeDef) <= FLASH_PAGE_SIZE, "CaliData_tTypeDef exceeds one page");

/* Function ------------------------------------------------------------------*/
static uint16_t CaliStore_Crc(const CaliData_tTypeDef *pData);
static uint8_t  CaliStore_Check(const CaliData_tTypeDef *pData);
static uint8_t  CaliStore_Program(const CaliData_tTypeDef *pData);

/**
 * @brief  从 Flash 读出标定数据并校验
 * @retval 0:读到有效数据  1:无有效数据(首次上电或数据损坏)
 */
uint8_t CaliStore_Load(void)
{
    const CaliData_tTypeDef *pFlash = (const CaliData_tTypeDef *)CALI_STORE_ADDR;

    caliValid = 0U;

    if (0U != CaliStore_Check(pFlash))
    {
        return 1U;
    }

    caliData  = *pFlash;
    caliValid = 1U; 

    return 0U;
}

/**
 * @brief  把标定数据写入 Flash(先擦后写，再回读验证)
 * @retval 0:成功  1:失败
 * @note   只能在电机停转时调用，见文件头说明
 */
uint8_t CaliStore_Save(float med_angle, int32_t gyrox_offset)
{
    caliData.magic        = CALI_STORE_MAGIC;
    caliData.version      = CALI_STORE_VERSION;
    caliData.length       = (uint16_t)sizeof(CaliData_tTypeDef);
    caliData.med_angle    = med_angle;
    caliData.gyrox_offset = gyrox_offset;
    caliData.reserved     = 0U;
    caliData.crc          = CaliStore_Crc(&caliData);

    if (0U != CaliStore_Program(&caliData))
    {
        caliValid = 0U;
        return 1U;
    }

    /* 回读验证：擦写失败时 Flash 可能停在半写状态，不回读就会以为存成功了 */
    if (0U != CaliStore_Check((const CaliData_tTypeDef *)CALI_STORE_ADDR))
    {
        caliValid = 0U;
        return 1U;
    }

    caliValid = 1U;

    return 0U;
}

/**
 * @brief  擦掉标定页，下次开机重新标定
 * @retval 0:成功  1:失败
 */
uint8_t CaliStore_Erase(void)
{
    FLASH_EraseInitTypeDef erase;
    uint32_t               pageError = 0U;
    HAL_StatusTypeDef      status;

    erase.TypeErase   = FLASH_TYPEERASE_PAGES;
    erase.PageAddress = CALI_STORE_ADDR;
    erase.NbPages     = 1U;

    HAL_FLASH_Unlock();
    status = HAL_FLASHEx_Erase(&erase, &pageError);
    HAL_FLASH_Lock();

    caliValid = 0U;

    return (HAL_OK == status) ? 0U : 1U;
}

/**
 * @brief  当前是否已有可用标定(来自 Flash 或本次开机刚标定完)
 */
uint8_t CaliStore_IsValid(void)
{
    return caliValid;
}

float CaliStore_GetMedAngle(void)
{
    return caliData.med_angle;
}

int32_t CaliStore_GetGyroxOffset(void)
{
    return caliData.gyrox_offset;
}

/**
 * @brief  计算校验值，覆盖 crc 字段之前的所有字节
 */
static uint16_t CaliStore_Crc(const CaliData_tTypeDef *pData)
{
    return CRC16((const uint8_t *)pData, (uint16_t)offsetof(CaliData_tTypeDef, crc));
}

/**
 * @brief  校验一份标定数据
 * @retval 0:有效  1:无效
 */
static uint8_t CaliStore_Check(const CaliData_tTypeDef *pData)
{
    if (CALI_STORE_MAGIC != pData->magic)
    {
        return 1U;
    }

    if (CALI_STORE_VERSION != pData->version)
    {
        return 1U;
    }

    if (sizeof(CaliData_tTypeDef) != pData->length)
    {
        return 1U;
    }

    return (CaliStore_Crc(pData) == pData->crc) ? 0U : 1U;
}

/**
 * @brief  擦除标定页并按半字写入
 * @retval 0:成功  1:失败
 */
static uint8_t CaliStore_Program(const CaliData_tTypeDef *pData)
{
    const uint8_t         *pByte = (const uint8_t *)pData;
    FLASH_EraseInitTypeDef erase;
    uint32_t               pageError = 0U;
    uint16_t               half;
    uint32_t               i;

    erase.TypeErase   = FLASH_TYPEERASE_PAGES;
    erase.PageAddress = CALI_STORE_ADDR;
    erase.NbPages     = 1U;

    HAL_FLASH_Unlock();

    if (HAL_OK != HAL_FLASHEx_Erase(&erase, &pageError))
    {
        HAL_FLASH_Lock();
        return 1U;
    }

    for (i = 0U; i < sizeof(CaliData_tTypeDef); i += 2U)
    {
        half = (uint16_t)pByte[i] | ((uint16_t)pByte[i + 1U] << 8);

        if (HAL_OK != HAL_FLASH_Program(FLASH_TYPEPROGRAM_HALFWORD, CALI_STORE_ADDR + i, half))
        {
            HAL_FLASH_Lock();
            return 1U;
        }
    }

    HAL_FLASH_Lock();

    return 0U;
}
