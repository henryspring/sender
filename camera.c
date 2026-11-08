#include "camera.h"
#include "mailbox_config.h"
#include "debug.h"
#include "ov5640.h"

#include <string.h>

#define OV5640_I2C_ADDRESS      0x78u
#define OV5640_JPEG_SCALE_REG   0x4407u

static I2C_HandleTypeDef *camera_i2c;
static DCMI_HandleTypeDef *camera_dcmi;
static OV5640_Object_t camera;
static volatile uint8_t frame_done;
static volatile uint8_t frame_error;
static uint32_t current_resolution = OV5640_R640x480;

/* DCMI DMA writes directly here. No second image buffer is allocated. */
__attribute__((aligned(4))) static uint8_t frame_buffer[CAMERA_BUFFER_BYTES];

static int32_t bus_init(void)
{
    return 0;
}

static int32_t bus_deinit(void)
{
    return 0;
}

static int32_t bus_tick(void)
{
    return (int32_t)HAL_GetTick();
}

static int32_t bus_write(uint16_t device, uint16_t reg, uint8_t *data, uint16_t count)
{
    if (HAL_I2C_Mem_Write(camera_i2c, device, reg, I2C_MEMADD_SIZE_16BIT,
                          data, count, 100u) != HAL_OK) {
        return -1;
    }
    return 0;
}

static int32_t bus_read(uint16_t device, uint16_t reg, uint8_t *data, uint16_t count)
{
    if (HAL_I2C_Mem_Read(camera_i2c, device, reg, I2C_MEMADD_SIZE_16BIT,
                         data, count, 100u) != HAL_OK) {
        return -1;
    }
    return 0;
}

static int set_quality(uint8_t scale)
{
    return HAL_I2C_Mem_Write(camera_i2c, OV5640_I2C_ADDRESS,
                             OV5640_JPEG_SCALE_REG, I2C_MEMADD_SIZE_16BIT,
                             &scale, 1u, 100u) == HAL_OK;
}

static int set_resolution(uint32_t resolution)
{
    if (current_resolution == resolution) {
        return 1;
    }

    if (OV5640_SetResolution(&camera, resolution) != OV5640_OK) {
        return 0;
    }

    current_resolution = resolution;
    HAL_Delay(100u);
    return 1;
}

/* A JPEG this small cannot be a valid OV5640 photo. */
#define CAMERA_MIN_JPEG_BYTES 1024u

/* Search ONLY bytes that DMA actually wrote, not the unused DMA buffer. */
static int find_jpeg(uint32_t valid_bytes, uint32_t *offset, uint32_t *length)
{
    if (valid_bytes > CAMERA_BUFFER_BYTES) {
        valid_bytes = CAMERA_BUFFER_BYTES;
    }
    if (valid_bytes < CAMERA_MIN_JPEG_BYTES) {
        return 0;
    }

    uint32_t start = 0u;
    while (start + 1u < valid_bytes) {
        if (frame_buffer[start] == 0xFFu && frame_buffer[start + 1u] == 0xD8u) {
            break;
        }
        start++;
    }
    if (start + 1u >= valid_bytes) {
        return 0;
    }

    for (uint32_t i = start + 2u; i + 1u < valid_bytes; i++) {
        if (frame_buffer[i] == 0xFFu && frame_buffer[i + 1u] == 0xD9u) {
            uint32_t jpeg_length = i + 2u - start;
            if (jpeg_length < CAMERA_MIN_JPEG_BYTES) {
                continue;  /* Reject false end markers such as the 22-byte result. */
            }
            *offset = start;
            *length = jpeg_length;
            return 1;
        }
    }
    return 0;
}

/* Temporary diagnostic: distinguish an empty DMA buffer from bad JPEG data. */
static void debug_capture_buffer(uint32_t dma_bytes_left, uint32_t valid_bytes)
{
    uint32_t nonzero = 0u;
    uint32_t first_nonzero = CAMERA_BUFFER_BYTES;
    uint32_t soi = CAMERA_BUFFER_BYTES;
    uint32_t eoi = CAMERA_BUFFER_BYTES;

    for (uint32_t i = 0u; i < valid_bytes; i++) {
        if (frame_buffer[i] != 0u) {
            nonzero++;
            if (first_nonzero == CAMERA_BUFFER_BYTES) {
                first_nonzero = i;
            }
        }
        if (i + 1u < valid_bytes) {
            if (soi == CAMERA_BUFFER_BYTES &&
                frame_buffer[i] == 0xFFu && frame_buffer[i + 1u] == 0xD8u) {
                soi = i;
            }
            if (eoi == CAMERA_BUFFER_BYTES &&
                frame_buffer[i] == 0xFFu && frame_buffer[i + 1u] == 0xD9u) {
                eoi = i;
            }
        }
    }

    Debug_Printf("DMA transferred=%lu, bytes left=%lu, nonzero=%lu, first=%ld\r\n",
                 (unsigned long)valid_bytes,
                 (unsigned long)dma_bytes_left,
                 (unsigned long)nonzero,
                 (long)(first_nonzero == CAMERA_BUFFER_BYTES ? -1L : (long)first_nonzero));
    Debug_Printf("JPEG SOI=%ld EOI=%ld (-1 means missing)\r\n",
                 (long)(soi == CAMERA_BUFFER_BYTES ? -1L : (long)soi),
                 (long)(eoi == CAMERA_BUFFER_BYTES ? -1L : (long)eoi));
    Debug_Printf("First 16: %02X %02X %02X %02X %02X %02X %02X %02X "
                 "%02X %02X %02X %02X %02X %02X %02X %02X\r\n",
                 frame_buffer[0], frame_buffer[1], frame_buffer[2], frame_buffer[3],
                 frame_buffer[4], frame_buffer[5], frame_buffer[6], frame_buffer[7],
                 frame_buffer[8], frame_buffer[9], frame_buffer[10], frame_buffer[11],
                 frame_buffer[12], frame_buffer[13], frame_buffer[14], frame_buffer[15]);

    if (valid_bytes >= 16u) {
        uint32_t i = valid_bytes - 16u;
        Debug_Printf("Last 16:  %02X %02X %02X %02X %02X %02X %02X %02X "
                     "%02X %02X %02X %02X %02X %02X %02X %02X\r\n",
                     frame_buffer[i], frame_buffer[i+1], frame_buffer[i+2], frame_buffer[i+3],
                     frame_buffer[i+4], frame_buffer[i+5], frame_buffer[i+6], frame_buffer[i+7],
                     frame_buffer[i+8], frame_buffer[i+9], frame_buffer[i+10], frame_buffer[i+11],
                     frame_buffer[i+12], frame_buffer[i+13], frame_buffer[i+14], frame_buffer[i+15]);
    }
}

static int capture_once(uint32_t *offset, uint32_t *length)
{
    frame_done = 0u;
    frame_error = 0u;
    memset(frame_buffer, 0, sizeof(frame_buffer));

    HAL_StatusTypeDef status = HAL_DCMI_Start_DMA(
        camera_dcmi,
        DCMI_MODE_SNAPSHOT,
        (uint32_t)(uintptr_t)frame_buffer,
        CAMERA_BUFFER_BYTES / 4u
    );

    Debug_Printf("DCMI start: status=%d, error=0x%08lX\r\n",
                 (int)status,
                 (unsigned long)HAL_DCMI_GetError(camera_dcmi));

    if (status != HAL_OK) {
        /* Leave the peripheral ready for the next capture attempt. */
        HAL_DCMI_Stop(camera_dcmi);
        return 0;
    }

    /* JPEG length varies; the DMA buffer may not fill before frame end. */
    __HAL_DCMI_ENABLE_IT(camera_dcmi, DCMI_IT_FRAME);

    uint32_t start_ms = HAL_GetTick();
    while (!frame_done && !frame_error) {
        if ((uint32_t)(HAL_GetTick() - start_ms) >= 5000u) {
            break;
        }
    }

    /* Log the capture state before Stop() alters the peripheral state. */
    Debug_Printf("DCMI result: done=%u, error=%u, HAL=0x%08lX\r\n",
                 (unsigned)frame_done,
                 (unsigned)frame_error,
                 (unsigned long)HAL_DCMI_GetError(camera_dcmi));

    /* Allow queued DMA writes to drain after the frame event, before stopping. */
    if (frame_done && !frame_error) {
        HAL_Delay(5u);
    }

    /* Read GPDMA's remaining byte count BEFORE HAL_DCMI_Stop aborts DMA. */
    uint32_t dma_bytes_left = camera_dcmi->DMA_Handle->Instance->CBR1 & DMA_CBR1_BNDT;
    uint32_t valid_bytes = (dma_bytes_left <= CAMERA_BUFFER_BYTES)
                         ? CAMERA_BUFFER_BYTES - dma_bytes_left : 0u;
    HAL_DCMI_Stop(camera_dcmi);

    if (!frame_done || frame_error) {
        Debug_Printf("Frame incomplete or error\r\n");
        debug_capture_buffer(dma_bytes_left, valid_bytes);
        return 0;
    }

    if (!find_jpeg(valid_bytes, offset, length)) {
        Debug_Printf("No complete, plausible JPEG in DMA data\r\n");
        debug_capture_buffer(dma_bytes_left, valid_bytes);
        return 0;
    }

    return 1;
}

int Camera_Init(I2C_HandleTypeDef *i2c, DCMI_HandleTypeDef *dcmi)
{
    camera_i2c = i2c;
    camera_dcmi = dcmi;

    /* Preserves the slow SCCB/I2C timing from the working camera prototype. */
    if (HAL_I2C_DeInit(camera_i2c) != HAL_OK) {
        return 0;
    }
    camera_i2c->Init.Timing = 0xF0526363u;
    if (HAL_I2C_Init(camera_i2c) != HAL_OK) {
        return 0;
    }

    OV5640_IO_t io = {0};
    io.Init = bus_init;
    io.DeInit = bus_deinit;
    io.Address = OV5640_I2C_ADDRESS;
    io.WriteReg = bus_write;
    io.ReadReg = bus_read;
    io.GetTick = bus_tick;

    if (OV5640_RegisterBusIO(&camera, &io) != OV5640_OK) {
        return 0;
    }

    int32_t result = OV5640_ERROR;
    for (int attempt = 0; attempt < 5 && result != OV5640_OK; attempt++) {
        result = OV5640_Init(&camera, OV5640_R640x480, OV5640_JPEG);
        if (result != OV5640_OK) {
            HAL_Delay(100u);
        }
    }
    if (result != OV5640_OK) {
        return 0;
    }

    /* Test: slow down the camera's parallel pixel output to reduce sampling errors. */
    if (OV5640_SetPCLK(&camera, OV5640_PCLK_7M) != OV5640_OK) {
        Debug_Printf("Could not set camera PCLK to 7 MHz\r\n");
        return 0;
    }
    Debug_Printf("Camera PCLK set to 7 MHz\r\n");

    camera_dcmi->Init.JPEGMode = DCMI_JPEG_ENABLE;
    camera_dcmi->Init.PCKPolarity = DCMI_PCKPOLARITY_RISING;
    camera_dcmi->Init.VSPolarity = DCMI_VSPOLARITY_HIGH;
    camera_dcmi->Init.HSPolarity = DCMI_HSPOLARITY_HIGH;

    if (HAL_DCMI_Init(camera_dcmi) != HAL_OK) {
        return 0;
    }

    current_resolution = OV5640_R640x480;
    return set_quality(0x20u);
}

int Camera_CaptureJPEG(const uint8_t **jpeg, uint32_t *size)
{
    if (jpeg == NULL || size == NULL) {
        return 0;
    }

    *jpeg = NULL;
    *size = 0u;

    struct CaptureSetting {
        uint32_t resolution;
        uint8_t scale;
    };
    static const struct CaptureSetting settings[] = {
        {OV5640_R640x480, 0x02u},  // Highest quality
        {OV5640_R640x480, 0x04u},  // Excellent fallback
        {OV5640_R640x480, 0x08u},  // Previous working quality
        {OV5640_R640x480, 0x10u}   // Smaller fallback
    };

    for (unsigned i = 0; i < sizeof(settings) / sizeof(settings[0]); i++) {
        if (!set_resolution(settings[i].resolution)) {
            continue;
        }
        if (!set_quality(settings[i].scale)) {
            continue;
        }

        HAL_Delay(100u);
        uint32_t offset = 0u;
        uint32_t length = 0u;
        if (!capture_once(&offset, &length)) {
            Debug_Printf("Camera capture failed, attempt %u\r\n", i + 1u);
            continue;
        }
        if (length > CAMERA_MAX_JPEG_BYTES) {
            Debug_Printf("JPEG too large: %lu bytes\r\n", (unsigned long)length);
            continue;
        }

        *jpeg = frame_buffer + offset;
        *size = length;
        Debug_Printf("JPEG ready: %lu bytes, %s, scale %02X\r\n",
                     (unsigned long)length,
                     settings[i].resolution == OV5640_R640x480 ? "640x480" : "480x272",
                     settings[i].scale);
        return 1;
    }

    Debug_Printf("Camera could not produce a JPEG <= %u bytes\r\n",
                 (unsigned)CAMERA_MAX_JPEG_BYTES);
    return 0;
}

void HAL_DCMI_FrameEventCallback(DCMI_HandleTypeDef *hdcmi)
{
    if (hdcmi == camera_dcmi) {
        frame_done = 1u;
    }
}

void HAL_DCMI_ErrorCallback(DCMI_HandleTypeDef *hdcmi)
{
    if (hdcmi == camera_dcmi) {
        frame_error = 1u;
    }
}
