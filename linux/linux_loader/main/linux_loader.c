
#include <stdio.h>
#include "esp_system.h"
#include "esp_partition.h"
#include "esp_log.h"
#include "esp_psram.h"
#include "esp_heap_caps.h"
#include "esp_mmu_map.h"

static const char *TAG = "linux_loader";

// Standard RISC-V Linux load address
#define LINUX_LOAD_ADDR     0x48010000
#define LINUX_ENTRY_OFFSET  0x00000000  // Kernel entry point offset

// Minimal SBI implementation
#define SBI_SET_TIMER 0x0
#define SBI_CONSOLE_PUTCHAR 0x1
#define SBI_CONSOLE_GETCHAR 0x2
#define SBI_SHUTDOWN 0x8

struct riscv_image_header {
    uint32_t code0;          /* Executable code */
    uint32_t code1;          /* Executable code */
    uint64_t text_offset;    /* Image load offset, little endian */
    uint64_t image_size;     /* Effective Image size, little endian */
    uint64_t flags;          /* kernel flags, little endian */
    uint32_t version;        /* Version of this header */
    uint32_t res1;           /* Reserved */
    uint64_t res2;           /* Reserved */
    uint64_t magic;          /* Magic number, little endian, "RISCV" */
    uint32_t magic2;         /* Magic number 2, little endian, "RSC\x05" */
    uint32_t res3;           /* Reserved for PE COFF offset */
};

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

IRAM_ATTR void *malloc_psram_executable(size_t size) {
    //Malloc from PSRAM
    void *raw_buffer = NULL;
    raw_buffer = heap_caps_malloc(size, MALLOC_CAP_SPIRAM);
    if (raw_buffer == NULL) {
        ESP_LOGE(TAG, "No mem for psram.");
        return NULL;
    }

    //Get the physical address for allocated memory
    esp_paddr_t psram_buf_paddr = 0;
    mmu_target_t out_target;
    ESP_ERROR_CHECK(esp_mmu_vaddr_to_paddr(raw_buffer, &psram_buf_paddr, &out_target));

    //Map the same physical pages to instruction bus
    const size_t low_paddr = psram_buf_paddr & ~(CONFIG_MMU_PAGE_SIZE - 1);// round down to page boundary
    const size_t high_paddr = (psram_buf_paddr + size + CONFIG_MMU_PAGE_SIZE - 1) &
                              ~(CONFIG_MMU_PAGE_SIZE - 1);// round up to page boundary
    const size_t map_size = high_paddr - low_paddr;
    void *mmap_ptr = NULL;
    ESP_ERROR_CHECK(esp_mmu_map(0, map_size, MMU_TARGET_PSRAM0, MMU_MEM_CAP_EXEC, 0, &mmap_ptr));
    esp_mmu_map_dump_mapped_blocks(stdout);

    //Adjust the mapped pointer to point to the beginning of the buffer
    void *exec_buf = mmap_ptr + (psram_buf_paddr - low_paddr);
    return raw_buffer;
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
    linux_load_addr = malloc_psram_executable(linux_partition->size);

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
        return;
    }

    ESP_LOGI(TAG, "Linux loaded successfully to PSRAM");

    const struct riscv_image_header* header = (const struct riscv_image_header*)linux_load_addr;

    ESP_LOGI(TAG, "=== RISC-V KERNEL IMAGE HEADER ===");
    ESP_LOGI(TAG, "code0: 0x%08lx", header->code0);
    ESP_LOGI(TAG, "code1: 0x%08lx", header->code1);
    ESP_LOGI(TAG, "text_offset: 0x%08llx (%llu bytes)", header->text_offset, header->text_offset);
    ESP_LOGI(TAG, "image_size: 0x%08llx (%llu bytes)", header->image_size, header->image_size);
    ESP_LOGI(TAG, "flags: 0x%08llx", header->flags);
    ESP_LOGI(TAG, "version: 0x%08lx", header->version);
    ESP_LOGI(TAG, "magic: 0x%016llx", header->magic);
    ESP_LOGI(TAG, "magic2: 0x%08lx", header->magic2);
    ESP_LOGI(TAG, "================================");

    // Verify magic numbers
    if (header->magic != 0x5643534952ULL) {  // "RISCV"
        ESP_LOGW(TAG, "Warning: Invalid RISC-V magic number!");
    }
    if (header->magic2 != 0x05435352UL) {   // "RSC\x05"
        ESP_LOGW(TAG, "Warning: Invalid RISC-V magic2 number!");
    }

    // Calculate the actual kernel entry point
    const void* kernel_entry = (const char*)linux_load_addr + header->text_offset;

    ESP_LOGI(TAG, "=== KERNEL ENTRY CALCULATION ===");
    ESP_LOGI(TAG, "Kernel Image base: %p", linux_load_addr);
    ESP_LOGI(TAG, "Text offset: 0x%llx (%llu bytes)", header->text_offset, header->text_offset);
    ESP_LOGI(TAG, "Actual kernel entry: %p", kernel_entry);
    ESP_LOGI(TAG, "===============================");

    // Prepare boot parameters according to RISC-V Linux boot protocol
    // a0 = hartid (hardware thread ID, usually 0 for single core)
    // a1 = device tree address (0 since DTB is embedded)
    unsigned long hartid = 0;      // hartid
    unsigned long dtb_addr = 0;    // DTB address (embedded in kernel)

    ESP_LOGI(TAG, "Setting up for kernel boot...");
    ESP_LOGI(TAG, "hartid: %lu, dtb_addr: %lu", hartid, dtb_addr);

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
