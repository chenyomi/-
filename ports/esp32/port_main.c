#include "yoomi/cell.h"

/*
 * One control tick. The host build calls this from tests; on ESP32 the
 * 1 kHz task below is the same call.
 */
int yoomi_port_step(yoomi_cell_t *cell, uint32_t dt_ms)
{
    cell_kick(cell);
    cell_tick(cell, dt_ms);
    return cell_fault(cell);
}

#if defined(ESP_PLATFORM)
#include "esp_timer.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

static void cell_task(void *arg)
{
    yoomi_cell_t *cell = arg;
    int64_t last = esp_timer_get_time();
    for (;;) {
        int64_t now = esp_timer_get_time();
        uint32_t dt = (uint32_t)((now - last) / 1000);
        if (dt == 0) {
            dt = 1;
        }
        last = now;
        (void)yoomi_port_step(cell, dt);
        vTaskDelay(pdMS_TO_TICKS(1));
    }
}

void app_main(void)
{
    static yoomi_cell_t cell;
    cell_init(&cell);
    xTaskCreate(cell_task, "yoomi", 8192, &cell, 10, 0);
}
#endif
