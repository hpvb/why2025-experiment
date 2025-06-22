
#include <stdio.h>
#include "esp_system.h"
#include "esp_partition.h"
#include "esp_log.h"
#include "esp_psram.h"
#include "esp_heap_caps.h"
#include "esp_mmu_map.h"  // For MMU mapping functions

static const char *TAG = "linux_loader";

// Standard RISC-V Linux load address
#define LINUX_LOAD_ADDR     0x80000000
#define LINUX_ENTRY_OFFSET  0x00000000  // Kernel entry point offset

// Minimal SBI implementation
#define SBI_SET_TIMER 0x0
#define SBI_CONSOLE_PUTCHAR 0x1
#define SBI_CONSOLE_GETCHAR 0x2
#define SBI_SHUTDOWN 0x8

// Simple SBI call handler (very minimal implementation)
void sbi_call_handler(unsigned long which, unsigned long arg0, unsigned long arg1, unsigned long arg2) {
    switch(which) {
        case SBI_CONSOLE_PUTCHAR:
            putchar((char)arg0);
            break;
        case SBI_SHUTDOWN:
            esp_restart();
            break;
        // Add other SBI calls as needed
        default:
            ESP_LOGW(TAG, "Unhandled SBI call: %lu", which);
            break;
    }
}

void app_main(void)
{
    ESP_LOGI(TAG, "Linux Loader Starting...");

    // Check PSRAM
    size_t psram_size = esp_psram_get_size();
    ESP_LOGI(TAG, "PSRAM size: %d bytes", psram_size);

    if (psram_size < 8 * 1024 * 1024) {  // Need at least 8MB
        ESP_LOGE(TAG, "Insufficient PSRAM for Linux boot!");
        return;
    }

    // Find Linux partition
    const esp_partition_t* linux_partition = esp_partition_find_first(
        ESP_PARTITION_TYPE_DATA, ESP_PARTITION_SUBTYPE_ANY, "linux");

    if (!linux_partition) {
        ESP_LOGE(TAG, "Linux partition not found!");
        return;
    }

    ESP_LOGI(TAG, "Found Linux partition at 0x%lx, size: 0x%lx",
             linux_partition->address, linux_partition->size);

    // Try to map PSRAM to the standard Linux load address if possible
    // Otherwise load to PSRAM and copy later
    void* linux_load_addr;

    // Check if we can directly use 0x80000000 (unlikely on ESP32)
    // For now, load to PSRAM and we'll handle MMU setup
    linux_load_addr = heap_caps_aligned_alloc(0x10000, linux_partition->size, MALLOC_CAP_SPIRAM);

    if (!linux_load_addr) {
        ESP_LOGE(TAG, "Failed to allocate PSRAM for Linux!");
        return;
    }

    ESP_LOGI(TAG, "Allocated %d bytes from PSRAM at %p",
             linux_partition->size, linux_load_addr);

    // Read Linux binary from partition
    esp_err_t ret = esp_partition_read(linux_partition, 0,
                                      linux_load_addr, linux_partition->size);

    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to read Linux partition: %s", esp_err_to_name(ret));
        heap_caps_free(linux_load_addr);
        return;
    }

    ESP_LOGI(TAG, "Linux loaded successfully to PSRAM");

    // CRITICAL: Map PSRAM to the virtual address Linux expects (0x80000000)
    void* virtual_addr = NULL;
    esp_err_t map_ret = esp_mmu_map((uint32_t)linux_load_addr, linux_partition->size,
                                   MMU_TARGET_PSRAM0,
                                   MMU_MEM_CAP_EXEC | MMU_MEM_CAP_READ | MMU_MEM_CAP_WRITE,
                                   0,  // flags
                                   &virtual_addr);

    if (map_ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to map Linux to virtual memory: %s", esp_err_to_name(map_ret));
        heap_caps_free(linux_load_addr);
        return;
    }

    ESP_LOGI(TAG, "Mapped PSRAM %p to virtual address %p", linux_load_addr, virtual_addr);

    // If the virtual address isn't 0x80000000, we need to try mapping specifically there
    if ((uintptr_t)virtual_addr != LINUX_LOAD_ADDR) {
        ESP_LOGW(TAG, "Virtual address %p doesn't match expected Linux load address 0x%x",
                 virtual_addr, LINUX_LOAD_ADDR);
        ESP_LOGW(TAG, "This may cause kernel boot issues");
    }

    // Prepare boot parameters according to RISC-V Linux boot protocol
    // a0 = hartid (hardware thread ID, usually 0 for single core)
    // a1 = device tree address (0 since DTB is embedded)
    unsigned long hartid = 0;      // hartid
    unsigned long dtb_addr = 0;    // DTB address (embedded in kernel)

    ESP_LOGI(TAG, "Setting up for kernel boot...");
    ESP_LOGI(TAG, "hartid: %lu, dtb_addr: %lu", hartid, dtb_addr);

    // Critical: Use the mapped virtual address, not the physical PSRAM address
    void* kernel_entry = virtual_addr;

    ESP_LOGI(TAG, "Kernel entry point: %p", kernel_entry);

    // Set up proper privilege mode transition
    // Linux runs in Supervisor mode, not Machine mode

    // 1. Set up mstatus for supervisor mode entry
    unsigned long mstatus_val = 0;
    mstatus_val |= (1UL << 11);  // MPP[1] = 1 (Supervisor mode)
    mstatus_val &= ~(1UL << 12); // MPP[2] = 0 (Supervisor mode)
    mstatus_val |= (1UL << 5);   // SPIE = 1 (enable supervisor interrupts after mret)

    // 2. Disable all machine-level interrupts
    __asm__ __volatile__("csrw mie, zero");

    // 3. Set machine status register
    __asm__ __volatile__("csrw mstatus, %0" : : "r"(mstatus_val));

    // 4. Set exception return address to kernel entry point
    __asm__ __volatile__("csrw mepc, %0" : : "r"(kernel_entry));

    ESP_LOGI(TAG, "Transitioning to Supervisor mode and jumping to Linux kernel...");

    // 5. Use mret to transition from Machine mode to Supervisor mode and jump to kernel
    __asm__ __volatile__(
        "mv a0, %0\n"      // Set hartid in a0
        "mv a1, %1\n"      // Set dtb address in a1
        "mret"             // Machine return - transitions to supervisor mode and jumps to mepc
        :
        : "r"(hartid), "r"(dtb_addr)
        : "a0", "a1", "memory"
    );

    // Should never reach here
    ESP_LOGE(TAG, "Kernel boot failed!");
}
