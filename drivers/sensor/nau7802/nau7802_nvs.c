
#include <zephyr/drivers/flash.h>
#include <zephyr/storage/flash_map.h>
#include <zephyr/fs/nvs.h>
#include "nau7802.h"
#include "nau7802_regmap.h"

/* Declare the module to logging submodule*/
LOG_MODULE_DECLARE(NAU7802, LOG_LEVEL_DBG);

#define NVS_PARTITION storage_partition
#define NVS_PARTITION_DEVICE FIXED_PARTITION_DEVICE(NVS_PARTITION)
#define NVS_PARTITION_OFFSET FIXED_PARTITION_OFFSET(NVS_PARTITION)

static struct nvs_fs fs;
#define CALIBRATION_ID 100 // Unique ID for storing calibration data
#define GAIN_ID 1
#define OFFSET_ID 2
#define FACTOR 1000 // Factor for scaling float values into integers2

int nau7802_nvs_init(void)
{
    LOG_DBG("REACHED nau7802_nvs_init");
    int rc = 0;
    struct flash_pages_info info;

    fs.flash_device = NVS_PARTITION_DEVICE;
    if (!device_is_ready(fs.flash_device))
    {
        LOG_ERR("Flash device %s is not ready\n", fs.flash_device->name);
        return -1;
    }

    fs.offset = NVS_PARTITION_OFFSET;
    rc = flash_get_page_info_by_offs(fs.flash_device, fs.offset, &info);
    if (rc)
    {
        LOG_ERR("Unable to get page info\n");
        return rc;
    }

    fs.sector_size = info.size;
    fs.sector_count = 2U;

    rc = nvs_mount(&fs);
    if (rc)
    {
        LOG_ERR("Flash Init failed\n");
        return rc;
    }
    return 0;
}

// Store calibration data
int store_calibration_data_nvs(int32_t offset, float32_t calibration_factor)
{

    // Initialize a const structure at declaration
    const struct calibration_data_v2 cal_data = {
        .zero_offset = offset,                    // Set offset value
        .calibration_factor = calibration_factor, // Set gain value, scaled by FACTOR
    };

    int ret = nvs_write(&fs, CALIBRATION_ID, (const void *)(&cal_data), sizeof(cal_data));
    if (ret != 0)
    {
        LOG_ERR("Failed to write calibration data to NVS, error code: %d", ret);
        return ret;
    }
    LOG_INF("Calibration data successfully written to NVS.");
    return 0;
}

// Load calibration data
int load_calibration_data_nvs(struct calibration_data_v2 *cal_data)
{
    int ret = nvs_read(&fs, CALIBRATION_ID, (void *)cal_data, sizeof(*cal_data));
    if (ret != 0)
    {
        LOG_ERR("Failed to read calibration data from NVS, error code: %d", ret);
        return ret;
    }

    // Log the offset and the recalculated gain
    LOG_INF("Calibration data successfully loaded from NVS: Offset = %d, Gain = %d",
            cal_data->zero_offset, cal_data->calibration_factor);
    return 0;
}
