#include "ab_confirm.h"

#include <stddef.h>
#include <string.h>

#include "main.h"
#include "stm32f4xx_hal_flash_ex.h"

#define APP_SLOT                   0U
#define BCB_ADDR                   0x08004000UL
#define BCB_SIZE                   0x00004000UL
#define BCB_MAGIC                  0x42434231UL
#define BCB_COMMIT                 0x434D4954UL
#define BCB_SLOT_NONE              0xFFFFFFFFUL

typedef struct
{
    uint32_t magic;
    uint32_t sequence;
    uint32_t confirmed_slot;
    uint32_t pending_slot;
    uint32_t trial_count;
    uint32_t image_size[2];
    uint32_t image_crc32[2];
    uint32_t record_crc32;
    uint32_t commit;
} bcb_record_t;

static uint32_t bcb_crc32(const uint8_t *data, uint32_t len)
{
    uint32_t crc = 0xFFFFFFFFUL;
    uint32_t i;
    uint32_t bit;

    for (i = 0U; i < len; i++)
    {
        crc ^= data[i];
        for (bit = 0U; bit < 8U; bit++)
        {
            crc = (crc & 1U) ? ((crc >> 1U) ^ 0xEDB88320UL) : (crc >> 1U);
        }
    }
    return crc ^ 0xFFFFFFFFUL;
}

static bool bcb_record_is_valid(const bcb_record_t *record)
{
    if ((record->magic != BCB_MAGIC) || (record->commit != BCB_COMMIT) ||
        (record->confirmed_slot > 1U) ||
        ((record->pending_slot != BCB_SLOT_NONE) && (record->pending_slot > 1U)))
    {
        return false;
    }
    return record->record_crc32 == bcb_crc32((const uint8_t *)record, offsetof(bcb_record_t, record_crc32));
}

static bool bcb_program_words(uint32_t address, const uint32_t *words, uint32_t count)
{
    HAL_StatusTypeDef status = HAL_OK;
    uint32_t i;

    if (HAL_FLASH_Unlock() != HAL_OK)
    {
        return false;
    }
    for (i = 0U; i < count; i++)
    {
        status = HAL_FLASH_Program(FLASH_TYPEPROGRAM_WORD, address + (i * 4U), words[i]);
        if (status != HAL_OK)
        {
            break;
        }
    }
    (void)HAL_FLASH_Lock();
    return status == HAL_OK;
}

app_boot_confirm_status_t app_boot_confirm(void)
{
    const bcb_record_t *latest = NULL;
    const bcb_record_t *record;
    bcb_record_t next;
    uint32_t address;

    for (address = BCB_ADDR; address + sizeof(bcb_record_t) <= (BCB_ADDR + BCB_SIZE);
         address += sizeof(bcb_record_t))
    {
        record = (const bcb_record_t *)address;
        if (bcb_record_is_valid(record) && ((latest == NULL) || (record->sequence > latest->sequence)))
        {
            latest = record;
        }
    }

    if (latest == NULL)
    {
        record = (const bcb_record_t *)BCB_ADDR;
        if (record->magic != BCB_MAGIC)
        {
            return APP_BOOT_CONFIRM_BAD_MAGIC;
        }
        if (record->commit != BCB_COMMIT)
        {
            return APP_BOOT_CONFIRM_BAD_COMMIT;
        }
        if ((record->confirmed_slot > 1U) ||
            ((record->pending_slot != BCB_SLOT_NONE) && (record->pending_slot > 1U)))
        {
            return APP_BOOT_CONFIRM_BAD_SLOT_FIELD;
        }
        return APP_BOOT_CONFIRM_BAD_CRC;
    }
    if (latest->pending_slot == BCB_SLOT_NONE)
    {
        return APP_BOOT_CONFIRM_OK;
    }
    if (latest->pending_slot != APP_SLOT)
    {
        return APP_BOOT_CONFIRM_PENDING_OTHER_SLOT;
    }

    next = *latest;
    next.sequence++;
    next.confirmed_slot = APP_SLOT;
    next.pending_slot = BCB_SLOT_NONE;
    next.trial_count = 0U;
    next.record_crc32 = bcb_crc32((const uint8_t *)&next, offsetof(bcb_record_t, record_crc32));
    next.commit = BCB_COMMIT;

    for (address = BCB_ADDR; address + sizeof(next) <= (BCB_ADDR + BCB_SIZE); address += sizeof(next))
    {
        if (*(const uint32_t *)address == 0xFFFFFFFFUL)
        {
            break;
        }
    }
    if (address + sizeof(next) > (BCB_ADDR + BCB_SIZE))
    {
        return APP_BOOT_CONFIRM_NO_SPACE;
    }

    if (!bcb_program_words(address, (const uint32_t *)&next, (sizeof(next) / 4U) - 1U))
    {
        return APP_BOOT_CONFIRM_WRITE_FAILED;
    }
    if (!bcb_program_words(address + offsetof(bcb_record_t, commit), &next.commit, 1U))
    {
        return APP_BOOT_CONFIRM_WRITE_FAILED;
    }
    return APP_BOOT_CONFIRM_OK;
}
