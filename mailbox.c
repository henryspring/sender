#include "mailbox.h"
#include "mailbox_config.h"
#include "camera.h"
#include "debug.h"
#include "lora_sender.h"

#include <stdint.h>
#include <stddef.h>

typedef enum {
    WAIT_FOR_CLOSED,
    ARMED,
    WAIT_FOR_CLOSE,
    SETTLING,
    LIGHT_WARMUP,
    WAIT_FOR_LINK,
    COOLDOWN
} MailboxState;

static MailboxState state;
static uint8_t door_open;
static uint8_t last_raw_open;
static uint8_t send_attempts;
static uint32_t raw_changed_ms;
static uint32_t state_started_ms;
static const uint8_t *pending_image;
static uint32_t pending_size;

static uint8_t read_door(void)
{
	return 0;
    //return HAL_GPIO_ReadPin(DOOR_PORT, DOOR_PIN) == DOOR_OPEN_LEVEL;
}

static void set_light(uint8_t on)
{
    GPIO_PinState level = on ? LED_ON_LEVEL :
                            (LED_ON_LEVEL == GPIO_PIN_SET ? GPIO_PIN_RESET : GPIO_PIN_SET);
    HAL_GPIO_WritePin(LED_PORT, LED_PIN, level);
}

static void update_door(uint32_t now)
{
    uint8_t raw_open = read_door();
    if (raw_open != last_raw_open) {
        last_raw_open = raw_open;
        raw_changed_ms = now;
    }

    if (door_open != raw_open &&
        (uint32_t)(now - raw_changed_ms) >= DOOR_DEBOUNCE_MS) {
        door_open = raw_open;
    }
}

static void start_cooldown(void)
{
    pending_image = NULL;
    pending_size = 0u;
    set_light(0u);
    state = COOLDOWN;
    state_started_ms = HAL_GetTick();
    Debug_Printf("Mailbox cooldown\r\n");
}

void Mailbox_Init(void)
{
    __HAL_RCC_GPIOB_CLK_ENABLE();
    __HAL_RCC_GPIOC_CLK_ENABLE();

    GPIO_InitTypeDef gpio = {0};

    gpio.Pin = LORA_AUX_PIN;
    gpio.Mode = GPIO_MODE_INPUT;
    gpio.Pull = GPIO_NOPULL;
    HAL_GPIO_Init(LORA_AUX_PORT, &gpio);

    HAL_GPIO_WritePin(LORA_M0_PORT, LORA_M0_PIN, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(LORA_M1_PORT, LORA_M1_PIN, GPIO_PIN_RESET);
    gpio.Pin = LORA_M0_PIN | LORA_M1_PIN;
    gpio.Mode = GPIO_MODE_OUTPUT_PP;
    gpio.Pull = GPIO_NOPULL;
    gpio.Speed = GPIO_SPEED_FREQ_LOW;
    HAL_GPIO_Init(LORA_M0_PORT, &gpio);

    gpio.Pin = DOOR_PIN;
    gpio.Mode = GPIO_MODE_INPUT;
    gpio.Pull = GPIO_PULLUP;
    HAL_GPIO_Init(DOOR_PORT, &gpio);

    set_light(0u);
    gpio.Pin = LED_PIN;
    gpio.Mode = GPIO_MODE_OUTPUT_PP;
    gpio.Pull = GPIO_NOPULL;
    gpio.Speed = GPIO_SPEED_FREQ_LOW;
    HAL_GPIO_Init(LED_PORT, &gpio);

    door_open = read_door();
    last_raw_open = door_open;
    raw_changed_ms = HAL_GetTick();
    //state = door_open ? WAIT_FOR_CLOSED : ARMED;
    state = SETTLING;
    state_started_ms = HAL_GetTick();

    Debug_Printf("Mailbox ready; door %s\r\n", door_open ? "open" : "closed");
}

void Mailbox_Task(void)
{
    LoRaSender_Poll();

    uint32_t now = HAL_GetTick();
    update_door(now);

    switch (state) {
    case WAIT_FOR_CLOSED:
        if (!door_open) {
            state = ARMED;
        }
        break;

    case ARMED:
        if (door_open) {
            Debug_Printf("Mailbox opened\r\n");
            state = WAIT_FOR_CLOSE;
        }
        break;

    case WAIT_FOR_CLOSE:
        if (!door_open) {
            state = SETTLING;
            state_started_ms = now;
        }
        break;

    case SETTLING:
        if (door_open) {
            state = WAIT_FOR_CLOSE;
        } else if ((uint32_t)(now - state_started_ms) >= CLOSED_SETTLE_MS) {
            set_light(1u);
            state = LIGHT_WARMUP;
            state_started_ms = now;
        }
        break;

    case LIGHT_WARMUP:
        if (door_open) {
            set_light(0u);
            state = WAIT_FOR_CLOSE;
        } else if ((uint32_t)(now - state_started_ms) >= LED_WARMUP_MS) {
            uint32_t size = 0u;
            const uint8_t *image = NULL;
            int captured = Camera_CaptureJPEG(&image, &size);
            set_light(0u);

            if (read_door()) {
                state = WAIT_FOR_CLOSE;
            } else if (captured) {
                pending_image = image;
                pending_size = size;
                send_attempts = 0u;
                state = WAIT_FOR_LINK;
                state_started_ms = HAL_GetTick();
            } else {
                Debug_Printf("No usable image captured\r\n");
                start_cooldown();
            }
        }
        break;

    case WAIT_FOR_LINK:
        if (LoRaSender_IsPaired()) {
            send_attempts++;
            if (LoRaSender_SendImage(pending_image, pending_size)) {
                Debug_Printf("Mailbox photo delivered\r\n");
                start_cooldown();
            } else if (send_attempts >= TRANSFER_ATTEMPTS) {
                Debug_Printf("Mailbox image transfer failed\r\n");
                start_cooldown();
            } else {
                state_started_ms = HAL_GetTick();
            }
        } else if ((uint32_t)(now - state_started_ms) >= LINK_WAIT_MS) {
            Debug_Printf("Mailbox link timeout\r\n");
            start_cooldown();
        }
        break;

    case COOLDOWN:
        if ((uint32_t)(now - state_started_ms) >= MAILBOX_COOLDOWN_MS &&
            !door_open) {
            state = ARMED;
            Debug_Printf("Mailbox armed\r\n");
        }
        break;
    }
}
