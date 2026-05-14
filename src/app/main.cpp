#include "control/hall_calibration.h"
#include "hardware/ws2812.h"

#include "storage/nvm_storage.h"
#include "control/motion_control.h"
#include "model/filament_state.h"
#include "hardware/adc_dma.h"
#include "platform/debug_log.h"
#include <stdio.h>
#include <string.h>

static bool parse_channel_command(const char *line, const char *verb, uint8_t *channel_id)
{
    if ((line == nullptr) || (verb == nullptr) || (channel_id == nullptr))
        return false;

    const size_t verb_len = strlen(verb);
    if (strncmp(line, verb, verb_len) != 0)
        return false;

    const char *p = line + verb_len;
    if (*p != ' ')
        return false;
    ++p;

    if ((p[0] < '0') || (p[0] > '3') || (p[1] != '\0'))
        return false;

    *channel_id = (uint8_t)(p[0] - '0');
    return true;
}

WS2812_class SYS_RGB;
WS2812_class RGBOUT[4];

void RGB_init()
{
    SYS_RGB.init(1, GPIOD, GPIO_Pin_1);
    RGBOUT[0].init(2, GPIOA, GPIO_Pin_11);
    RGBOUT[1].init(2, GPIOA, GPIO_Pin_8);
    RGBOUT[2].init(2, GPIOB, GPIO_Pin_1);
    RGBOUT[3].init(2, GPIOB, GPIO_Pin_0);
}

void RGB_update()
{
    if (!(SYS_RGB.is_dirty() ||
          RGBOUT[0].is_dirty() || RGBOUT[1].is_dirty() ||
          RGBOUT[2].is_dirty() || RGBOUT[3].is_dirty()))
        return;

    static uint32_t last = 0u;

    uint32_t min_gap = time_hw_tpms;
    if (!min_gap) min_gap = 1u;

    const uint32_t now = time_ticks32();
    if (last != 0u && (uint32_t)(now - last) < min_gap)
        return;

    last = now;

    SYS_RGB.updata();
    RGBOUT[0].updata();
    RGBOUT[1].updata();
    RGBOUT[2].updata();
    RGBOUT[3].updata();
}

static uint8_t g_fil_dirty = 0;
static uint8_t g_loaded_ch = 0xFF;
static uint8_t g_state_dirty = 0;

static void write_runtime_state()
{
    char out[384];
    int n = 0;

    n += snprintf(out + n, sizeof(out) - (size_t)n, "STATE {\"loaded\":%d,\"channels\":[", (g_loaded_ch < 4u) ? (int)g_loaded_ch : -1);

    for (uint8_t ch = 0; ch < 4u && n > 0 && (size_t)n < sizeof(out); ch++)
    {
        uint8_t sr = 0u, sg = 0u, sb = 0u;
        uint8_t or_ = 0u, og = 0u, ob = 0u;
        const uint8_t ks = Motion_control_key_state(ch);
        const uint8_t sw1 = ((ks == 1u) || (ks == 2u)) ? 1u : 0u;
        const uint8_t sw2 = ((ks == 1u) || (ks == 3u)) ? 1u : 0u;
        (void)RGBOUT[ch].get_RGB(0u, &sr, &sg, &sb);
        (void)RGBOUT[ch].get_RGB(1u, &or_, &og, &ob);

        n += snprintf(out + n, sizeof(out) - (size_t)n,
                      "%s{\"inserted\":%u,\"buffer\":\"%s\",\"sw1\":%u,\"sw2\":%u,\"status\":\"#%02X%02X%02X\",\"online\":\"#%02X%02X%02X\"}",
                      (ch == 0u) ? "" : ",",
                      filament_channel_inserted[ch] ? 1u : 0u,
                      Motion_control_buffer_mode_name(ch),
                      (unsigned)sw1,
                      (unsigned)sw2,
                      (unsigned)sr, (unsigned)sg, (unsigned)sb,
                      (unsigned)or_, (unsigned)og, (unsigned)ob);
    }

    if (n < 0) return;
    if ((size_t)n < sizeof(out))
        n += snprintf(out + n, sizeof(out) - (size_t)n, "]}\n");

    if (n > 0)
    {
        if ((size_t)n > sizeof(out)) n = (int)sizeof(out);
        Debug_log_write_num(out, n);
    }
}

static void state_report_run()
{
    static uint32_t last_ticks = 0u;

    uint32_t tpm = time_hw_tpms;
    if (!tpm) tpm = 1u;

    const uint32_t now = time_ticks32();
    const uint32_t interval = 500u * tpm;
    if (last_ticks != 0u && (uint32_t)(now - last_ticks) < interval)
        return;

    last_ticks = now;
    write_runtime_state();
}

static inline void ram_to_flashinfo(uint8_t fil, Flash_FilamentInfo* o)
{
    const _filament* f = &ams[BAMBU_BUS_AMS_NUM].filament[fil];

    memcpy(o->filament_id, f->filament_id, sizeof(o->filament_id));
    o->color_R = f->color_R;
    o->color_G = f->color_G;
    o->color_B = f->color_B;
    o->color_A = f->color_A;
    o->temperature_min = f->temperature_min;
    o->temperature_max = f->temperature_max;
    memcpy(o->name, f->name, sizeof(o->name));
}

static inline void flashinfo_to_ram(uint8_t fil, const Flash_FilamentInfo* i)
{
    _filament* f = &ams[BAMBU_BUS_AMS_NUM].filament[fil];

    memcpy(f->filament_id, i->filament_id, sizeof(i->filament_id));
    f->color_R = i->color_R;
    f->color_G = i->color_G;
    f->color_B = i->color_B;
    f->color_A = i->color_A;
    f->temperature_min = i->temperature_min;
    f->temperature_max = i->temperature_max;

    memset(f->name, 0, sizeof(f->name));
    memcpy(f->name, i->name, sizeof(i->name));
    f->name[sizeof(f->name) - 1u] = 0;
}

bool ams_datas_read()
{
    bool any = false;

    for (uint8_t fil = 0; fil < 4u; fil++)
    {
        Flash_FilamentInfo fi;
        if (Flash_AMS_filament_read(fil, &fi))
        {
            flashinfo_to_ram(fil, &fi);
            any = true;
        }
    }

    return any;
}

void ams_datas_set_need_to_save()
{
    g_fil_dirty = 0x0Fu;
}

void ams_datas_set_need_to_save_filament(uint8_t filament_idx)
{
    if (filament_idx >= 4u) return;
    g_fil_dirty |= (uint8_t)(1u << filament_idx);
}

void ams_state_set_loaded(uint8_t filament_ch)
{
    if (filament_ch >= 4u) return;
    if (g_loaded_ch != 0xFFu) return;
    g_loaded_ch = filament_ch;
    g_state_dirty = 1u;
}

void ams_state_set_unloaded(uint8_t filament_ch)
{
    if (g_loaded_ch == 0xFFu) return;
    if (filament_ch < 4u && g_loaded_ch != filament_ch) return;
    g_loaded_ch = 0xFFu;
    g_state_dirty = 1u;
}

uint8_t ams_state_get_loaded(void)
{
    return g_loaded_ch;
}

static void ams_state_save_run()
{
    if (!g_state_dirty) return;

    if (Flash_AMS_state_write(g_loaded_ch))
        g_state_dirty = 0u;
}

void ams_datas_save_run()
{
    if (!g_fil_dirty) return;

    uint8_t fil = 0xFFu;
    for (uint8_t i = 0; i < 4u; i++)
    {
        if (g_fil_dirty & (uint8_t)(1u << i))
        {
            fil = i;
            break;
        }
    }

    if (fil == 0xFFu) return;

    Flash_FilamentInfo now;
    ram_to_flashinfo(fil, &now);

    if (Flash_AMS_filament_write(fil, &now))
        g_fil_dirty &= (uint8_t)~(1u << fil);
}

int main(void)
{
    SystemInit();
    SystemCoreClockUpdate();
    time_hw_init();

    __enable_irq();

    WWDG_DeInit();
    RCC_APB1PeriphClockCmd(RCC_APB1Periph_WWDG, DISABLE);
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_AFIO, ENABLE);

    NVIC_PriorityGroupConfig(NVIC_PriorityGroup_1);
    GPIO_PinRemapConfig(GPIO_Remap_PD01, ENABLE);

    RGB_init();
    delay(10);

    SYS_RGB.set_RGB(0x10, 0x00, 0x00, 0);
    for (int i = 0; i < 4; i++) RGBOUT[i].set_RGB(0, 0, 0, 0);
    RGB_update();
    delay(50);

    Debug_log_init();
    ams_init();
    Flash_saves_init();

    ADC_DMA_init();
    ADC_DMA_wait_full();

    MC_PULL_calibration_boot();
    ams_datas_read();

    Motion_control_init();

    SYS_RGB.set_RGB(0x00, 0x10, 0x00, 0);
    RGB_update();

    Debug_log_write("START OFFLINE\n");

    while (1)
    {
        char line[32];
        if (Debug_log_readline(line, (int)sizeof(line)))
        {
            uint8_t channel_id = 0u;
            bool accepted = false;

            if (strcmp(line, "STATE") == 0)
            {
                write_runtime_state();
                accepted = true;
            }
            else if (parse_channel_command(line, "INPUT", &channel_id))
                accepted = Motion_control_uart_input(channel_id);
            else if (parse_channel_command(line, "OUTPUT", &channel_id))
                accepted = Motion_control_uart_output(channel_id);

            if (!accepted)
                Debug_log_write("ERR\n");
        }

        ams_datas_save_run();
        ams_state_save_run();
        Motion_control_run(0);

        if (Motion_control_uart_take_done())
            Debug_log_write("DONE\n");

        RGB_update();
        state_report_run();
    }
}
