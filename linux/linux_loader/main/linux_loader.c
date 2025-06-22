#include <stdio.h>
#include "esp_system.h"
#include "esp_partition.h"
#include "esp_log.h"
#include "soc/rtc.h"

static const char *TAG = "linux_loader";

void app_main(void)
{
    ESP_LOGI(TAG, "Linux Loader Starting...");
    
    const esp_partition_t* linux_partition = esp_partition_find_first(
        ESP_PARTITION_TYPE_DATA, ESP_PARTITION_SUBTYPE_ANY, "linux");
    
    if (linux_partition == NULL) {
        ESP_LOGE(TAG, "Linux partition not found!");
        return;
    }
    
    ESP_LOGI(TAG, "Found Linux partition at 0x%lx, size: 0x%lx", 
             linux_partition->address, linux_partition->size);
    
    void* linux_load_addr = (void*)0x40000000;
    
    ESP_LOGI(TAG, "Loading Linux binary to 0x%p...", linux_load_addr);
    
    esp_err_t ret = esp_partition_read(linux_partition, 0, 
                                      linux_load_addr, linux_partition->size);
    
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to read Linux partition: %s", esp_err_to_name(ret));
        return;
    }
    
    ESP_LOGI(TAG, "Linux loaded successfully. Jumping to kernel...");
    
    __asm__ __volatile__(
        "li t0, 0x40000000\n"  // Linux entry point
        "jr t0"                // Jump to Linux
        ::: "t0"
    );
}
