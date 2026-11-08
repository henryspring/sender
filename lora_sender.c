#include "lora_sender.h"
#include "lora_proto.h"
#include "mailbox_config.h"
#include "debug.h"

#include <string.h>

#define RADIO_RING_SIZE    1024u
#define ACK_TIMEOUT_MS     6000u
#define HELLO_INTERVAL_MS  2500u
#define AUX_TIMEOUT_MS     2000u
#define MAX_TRIES          8

static UART_HandleTypeDef *radio_uart;
static uint8_t rx_byte;
static volatile uint16_t rx_head;
static volatile uint16_t rx_tail;
static volatile uint32_t rx_overruns;
static uint8_t rx_ring[RADIO_RING_SIZE];
static LoraParser parser;
static uint8_t paired;
static uint8_t next_image_id;
static uint32_t last_hello_ms;

static int radio_pop(void)
{
    if (rx_head == rx_tail) {
        return -1;
    }

    uint8_t value = rx_ring[rx_tail];
    rx_tail = (uint16_t)((rx_tail + 1u) & (RADIO_RING_SIZE - 1u));
    return value;
}

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
    if (huart != radio_uart) {
        return;
    }

    uint16_t next = (uint16_t)((rx_head + 1u) & (RADIO_RING_SIZE - 1u));
    if (next == rx_tail) {
        rx_overruns++;
    } else {
        rx_ring[rx_head] = rx_byte;
        rx_head = next;
    }

    HAL_UART_Receive_IT(radio_uart, &rx_byte, 1u);
}

void HAL_UART_ErrorCallback(UART_HandleTypeDef *huart)
{
    if (huart == radio_uart) {
        HAL_UART_Receive_IT(radio_uart, &rx_byte, 1u);
    }
}

static int wait_aux(uint32_t timeout_ms)
{
    uint32_t started = HAL_GetTick();
    while (HAL_GPIO_ReadPin(LORA_AUX_PORT, LORA_AUX_PIN) == GPIO_PIN_RESET) {
        if ((uint32_t)(HAL_GetTick() - started) >= timeout_ms) {
            return 0;
        }
        HAL_Delay(1u);
    }

    HAL_Delay(25u);
    return 1;
}

static int query_radio(const uint8_t *command, uint16_t command_size,
                       uint8_t *reply, uint16_t reply_size)
{
    if (HAL_UART_Transmit(radio_uart, (uint8_t *)command,
                          command_size, 1000u) != HAL_OK) {
        return 0;
    }
    return HAL_UART_Receive(radio_uart, reply, reply_size, 1500u) == HAL_OK;
}

static void configure_radio(void)
{
    const uint8_t read_command[3] = {0xC1u, 0xC1u, 0xC1u};
    const uint8_t expected[6] = {
        0xC0u, 0x00u, 0x00u, E32_TARGET_SPED,
        E32_TARGET_CHANNEL, E32_TARGET_OPTION
    };
    uint8_t current[6];

    HAL_GPIO_WritePin(LORA_M0_PORT, LORA_M0_PIN, GPIO_PIN_SET);
    HAL_GPIO_WritePin(LORA_M1_PORT, LORA_M1_PIN, GPIO_PIN_SET);
    HAL_Delay(150u);

    if (!wait_aux(3000u)) {
        Debug_Printf("E32: AUX timeout in config mode\r\n");
        goto normal_mode;
    }
    if (!query_radio(read_command, sizeof(read_command), current, sizeof(current))) {
        Debug_Printf("E32: could not read settings\r\n");
        goto normal_mode;
    }
    if (current[0] != 0xC0u && current[0] != 0xC1u) {
        Debug_Printf("E32: unsupported config response\r\n");
        goto normal_mode;
    }

    if (memcmp(current + 1, expected + 1, 5u) != 0) {
#if E32_CONFIG_ON_BOOT
        Debug_Printf("E32: configuring radio\r\n");
        if (HAL_UART_Transmit(radio_uart, (uint8_t *)expected,
                              sizeof(expected), 1000u) != HAL_OK) {
            goto normal_mode;
        }
        HAL_Delay(100u);
        if (!wait_aux(3000u)) {
            goto normal_mode;
        }

        /* Some legacy E32 firmware echoes the write response. */
        uint8_t echo[6];
        HAL_UART_Receive(radio_uart, echo, sizeof(echo), 250u);

        if (!query_radio(read_command, sizeof(read_command),
                         current, sizeof(current)) ||
            memcmp(current + 1, expected + 1, 5u) != 0) {
            Debug_Printf("E32: config readback failed\r\n");
        } else {
            Debug_Printf("E32: configuration verified\r\n");
        }
#else
        Debug_Printf("E32: settings differ; read-only mode\r\n");
#endif
    }

normal_mode:
    HAL_GPIO_WritePin(LORA_M0_PORT, LORA_M0_PIN, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(LORA_M1_PORT, LORA_M1_PIN, GPIO_PIN_RESET);
    HAL_Delay(150u);
    if (!wait_aux(3000u)) {
        Debug_Printf("E32: AUX timeout returning to normal\r\n");
    }
}

static int send_frame(uint8_t flags, uint8_t id, uint16_t seq,
                      const uint8_t *payload, uint8_t length)
{
    uint8_t bytes[LORA_FRAME_MAX];
    uint16_t frame_size = (uint16_t)lora_build_frame(bytes, flags, id,
                                                     seq, payload, length);
    if (!wait_aux(AUX_TIMEOUT_MS)) {
        return 0;
    }
    if (HAL_UART_Transmit(radio_uart, bytes, frame_size, 3000u) != HAL_OK) {
        return 0;
    }
    HAL_Delay(5u);
    return wait_aux(AUX_TIMEOUT_MS);
}

static int wait_response(uint8_t flags, uint8_t id, uint16_t seq,
                         int check_seq, uint8_t length, int check_length,
                         LoraFrame *reply, uint32_t timeout_ms)
{
    uint32_t started = HAL_GetTick();
    while ((uint32_t)(HAL_GetTick() - started) < timeout_ms) {
        int value;
        while ((value = radio_pop()) >= 0) {
            if (!lora_parser_feed(&parser, (uint8_t)value, reply)) {
                continue;
            }
            if (reply->flags != flags || reply->id != id) {
                continue;
            }
            if (check_seq && reply->seq != seq) {
                continue;
            }
            if (check_length && reply->len != length) {
                continue;
            }
            return 1;
        }
        HAL_Delay(1u);
    }
    return 0;
}

void LoRaSender_Init(UART_HandleTypeDef *uart)
{
    radio_uart = uart;
    rx_head = 0u;
    rx_tail = 0u;
    memset(&parser, 0, sizeof(parser));
    paired = 0u;
    next_image_id = 0u;

    configure_radio();
    if (HAL_UART_Receive_IT(radio_uart, &rx_byte, 1u) != HAL_OK) {
        Debug_Printf("E32: UART receive could not start\r\n");
    }
    last_hello_ms = HAL_GetTick() - HELLO_INTERVAL_MS;
}

void LoRaSender_Poll(void)
{
    int value;
    LoraFrame frame;

    while ((value = radio_pop()) >= 0) {
        if (lora_parser_feed(&parser, (uint8_t)value, &frame) &&
            frame.flags == (LORA_F_HELLO | LORA_F_ACK)) {
            if (!paired) {
                Debug_Printf("LoRa receiver paired\r\n");
            }
            paired = 1u;
        }
    }

    uint32_t now = HAL_GetTick();
    if (!paired && (uint32_t)(now - last_hello_ms) >= HELLO_INTERVAL_MS) {
        last_hello_ms = now;
        send_frame(LORA_F_HELLO, 0u, 0u, NULL, 0u);
    }
}

int LoRaSender_IsPaired(void)
{
    return paired != 0u;
}

static int send_packet(const uint8_t *image, uint32_t image_size,
                       uint8_t image_id, uint16_t packet_count,
                       uint16_t seq, uint8_t request_ack)
{
    uint32_t offset = (uint32_t)seq * LORA_PAYLOAD;
    uint32_t remaining = image_size - offset;
    uint8_t length = (uint8_t)(remaining > LORA_PAYLOAD ? LORA_PAYLOAD : remaining);
    uint8_t flags = 0u;

    if (seq == packet_count - 1u) {
        flags |= LORA_F_END;
    }
    if (request_ack) {
        flags |= LORA_F_REQ;
    }
    return send_frame(flags, image_id, seq, image + offset, length);
}

static int send_missing(const uint8_t *image, uint32_t image_size,
                        uint8_t image_id, uint16_t packet_count,
                        uint16_t base, uint8_t count, uint32_t have)
{
    int last_missing = -1;
    for (int i = 0; i < count; i++) {
        if ((have & (1uL << i)) == 0u) {
            last_missing = i;
        }
    }

    for (int i = 0; i < count; i++) {
        if ((have & (1uL << i)) != 0u) {
            continue;
        }
        if (!send_packet(image, image_size, image_id, packet_count,
                         (uint16_t)(base + i), i == last_missing)) {
            return 0;
        }
    }
    return 1;
}

int LoRaSender_SendImage(const uint8_t *image, uint32_t size)
{
    if (!paired || image == NULL || size == 0u || size > CAMERA_MAX_JPEG_BYTES) {
        return 0;
    }

    uint16_t packet_count = (uint16_t)((size + LORA_PAYLOAD - 1u) / LORA_PAYLOAD);
    uint8_t image_id = ++next_image_id;
    uint32_t started = HAL_GetTick();
    uint8_t syn[10];
    LoraFrame response;

    lora_put32(syn, size);
    syn[4] = (uint8_t)(packet_count >> 8);
    syn[5] = (uint8_t)packet_count;
    lora_put32(syn + 6, lora_crc32(image, size));

    Debug_Printf("Sending image %u: %lu bytes, %u packets\r\n",
                 (unsigned)image_id, (unsigned long)size, (unsigned)packet_count);

    int accepted = 0;
    for (int tries = 0; tries < 10 && !accepted; tries++) {
        if (!send_frame(LORA_F_SYN, image_id, 0u, syn, sizeof(syn))) {
            break;
        }
        accepted = wait_response(LORA_F_SYN | LORA_F_ACK, image_id, 0u,
                                 0, 0u, 0, &response, ACK_TIMEOUT_MS);
    }
    if (!accepted) {
        Debug_Printf("LoRa: no SYN ACK\r\n");
        paired = 0u;
        return 0;
    }

    for (uint16_t base = 0; base < packet_count; base += LORA_WINDOW) {
        uint32_t remaining = (uint32_t)packet_count - base;
        uint8_t count = (uint8_t)(remaining < LORA_WINDOW ? remaining : LORA_WINDOW);
        uint32_t full = count == 32u ? 0xFFFFFFFFu : ((1uL << count) - 1u);
        uint32_t have = 0u;
        int done = 0;

        if (!send_missing(image, size, image_id, packet_count, base, count, have)) {
            Debug_Printf("LoRa: packet write failed\r\n");
            paired = 0u;
            return 0;
        }

        for (int tries = 0; tries < MAX_TRIES; tries++) {
            int got_ack = wait_response(LORA_F_ACK, image_id, base, 1, 4u, 1,
                                        &response, ACK_TIMEOUT_MS);
            if (got_ack) {
                have = lora_get32(response.payload) & full;
                if (have == full) {
                    done = 1;
                    break;
                }
                if (!send_missing(image, size, image_id, packet_count,
                                  base, count, have)) {
                    break;
                }
            } else {
                if (!send_packet(image, size, image_id, packet_count,
                                 (uint16_t)(base + count - 1u), 1u)) {
                    break;
                }
            }
        }
        if (!done) {
            Debug_Printf("LoRa: window %u failed\r\n", (unsigned)base);
            paired = 0u;
            return 0;
        }
    }

    for (int tries = 0; tries < MAX_TRIES; tries++) {
        if (wait_response(LORA_F_ACK | LORA_F_END, image_id, 0u,
                          0, 1u, 1, &response, ACK_TIMEOUT_MS)) {
            if (response.payload[0] == 0u) {
                Debug_Printf("Image confirmed in %lu ms\r\n",
                             (unsigned long)(HAL_GetTick() - started));
                return 1;
            }
            Debug_Printf("LoRa: receiver CRC32 mismatch\r\n");
            break;
        }
        if (!send_packet(image, size, image_id, packet_count,
                         (uint16_t)(packet_count - 1u), 1u)) {
            break;
        }
    }

    Debug_Printf("LoRa: transfer not confirmed\r\n");
    paired = 0u;
    return 0;
}
