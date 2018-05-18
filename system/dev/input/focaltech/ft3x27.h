#pragma once

#include <threads.h>

#include <ddk/device.h>
#include <ddk/protocol/gpio.h>
#include <ddk/protocol/test.h>
#include <ddktl/device.h>
#include <ddktl/protocol/hidbus.h>
#include <ddk/protocol/i2c.h>
#include <ddktl/protocol/test.h>
#include <fbl/mutex.h>
#include <fbl/unique_ptr.h>
#include <hid/ft3x27.h>

#include <lib/zx/interrupt.h>
#include <zircon/compiler.h>
#include <zircon/types.h>

#define FT_INT_PIN            0
#define FT_RESET_PIN          1

#define FTS_REG_CURPOINT                    0x02
#define FTS_REG_FINGER_START                0x03
#define FTS_REG_INT_CNT                     0x8F
#define FTS_REG_FLOW_WORK_CNT               0x91
#define FTS_REG_WORKMODE                    0x00
#define FTS_REG_WORKMODE_FACTORY_VALUE      0x40
#define FTS_REG_WORKMODE_WORK_VALUE         0x00
#define FTS_REG_ESDCHECK_DISABLE            0x8D
#define FTS_REG_CHIP_ID                     0xA3
#define FTS_REG_CHIP_ID2                    0x9F
#define FTS_REG_POWER_MODE                  0xA5
#define FTS_REG_POWER_MODE_SLEEP_VALUE      0x03
#define FTS_REG_FW_VER                      0xA6
#define FTS_REG_VENDOR_ID                   0xA8
#define FTS_REG_LCD_BUSY_NUM                0xAB
#define FTS_REG_FACE_DEC_MODE_EN            0xB0
#define FTS_REG_FACE_DEC_MODE_STATUS        0x01
#define FTS_REG_IDE_PARA_VER_ID             0xB5
#define FTS_REG_IDE_PARA_STATUS             0xB6
#define FTS_REG_GLOVE_MODE_EN               0xC0
#define FTS_REG_COVER_MODE_EN               0xC1
#define FTS_REG_CHARGER_MODE_EN             0x8B
#define FTS_REG_GESTURE_EN                  0xD0
#define FTS_REG_GESTURE_OUTPUT_ADDRESS      0xD3
#define FTS_REG_MODULE_ID                   0xE3
#define FTS_REG_LIC_VER                     0xE4
#define FTS_REG_ESD_SATURATE                0xED

namespace ft {
class Ft3x27Device : public ddk::Device<Ft3x27Device, ddk::Unbindable>,
                     public ddk::HidBusProtocol<Ft3x27Device> {
  public:
    Ft3x27Device(zx_device_t* device);


    static zx_status_t Create(zx_device_t* device);

    void DdkRelease();
    void DdkUnbind();

    void HidBusStop();
    zx_status_t HidBusGetDescriptor(uint8_t desc_type, void** data, size_t* len);
    zx_status_t HidBusGetReport(uint8_t rpt_type, uint8_t rpt_id, void* data,
                                 size_t len, size_t* out_len);
    zx_status_t HidBusSetReport(uint8_t rpt_type, uint8_t rpt_id, void* data,
                                 size_t len);
    zx_status_t HidBusGetIdle(uint8_t rpt_id, uint8_t* duration);
    zx_status_t HidBusSetIdle(uint8_t rpt_id, uint8_t duration);
    zx_status_t HidBusGetProtocol(uint8_t* protocol);
    zx_status_t HidBusSetProtocol(uint8_t protocol);

    zx_status_t HidBusStart(ddk::HidBusIfcProxy proxy) __TA_EXCLUDES(lock_);

    zx_status_t HidBusQuery(uint32_t options, hid_info_t* info) __TA_EXCLUDES(lock_);

  private:
    //Only one I2c channel is passed to this driver, so index should always
    // be zero.
    static constexpr uint32_t kI2cIndex = 0;
    // Number of reports this device can report simultaneously
    static constexpr uint32_t kMaxPoints = 5;
    // Size of each individual report (on the i2c bus)
    static constexpr uint32_t kFingerRptSize = 6; //Size of each Touch report

    zx_status_t InitPdev();
    uint8_t i2c_buf_[kMaxPoints * kFingerRptSize + 1];
    uint8_t Read(uint8_t addr);
    uint8_t Read(uint8_t addr, uint8_t len);
    int Thread();
    int TestThread();

    void ParseReport(ft3x27_finger_t* rpt, uint8_t* buf);

    gpio_protocol_t gpio_;
    zx::interrupt irq_;

    i2c_protocol_t i2c_;
    platform_device_protocol_t pdev_;

    thrd_t thread_;
    fbl::atomic<bool> running_;

    thrd_t test_thread_;

    fbl::Mutex lock_;
    ddk::HidBusIfcProxy proxy_ __TA_GUARDED(lock_);
};
}