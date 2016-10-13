// Copyright 2016 The Fuchsia Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.



// device descriptor for USB 2.0 root hub
// represented as a byte array to avoid endianness issues
static const uint8_t dwc_rh_descriptor[sizeof(usb_device_descriptor_t)] = {
    sizeof(usb_device_descriptor_t),    // bLength
    USB_DT_DEVICE,                      // bDescriptorType
    0x00, 0x02,                         // bcdUSB = 2.0
    USB_CLASS_HUB,                      // bDeviceClass
    0,                                  // bDeviceSubClass
    0,                                  // bDeviceProtocol = Single TT
    64,                                 // bMaxPacketSize0
    0xD1, 0x18,                         // idVendor = 0x18D1 (Google)
    0x02, 0xA0,                         // idProduct = 0xA002
    0x00, 0x01,                         // bcdDevice = 1.0
    0,                                  // iManufacturer
    1,                                  // iProduct
    0,                                  // iSerialNumber
    1,                                  // bNumConfigurations
};

#define CONFIG_DESC_SIZE sizeof(usb_configuration_descriptor_t) + \
                         sizeof(usb_interface_descriptor_t) + \
                         sizeof(usb_endpoint_descriptor_t)

static const uint8_t dwc_rh_config_desc[CONFIG_DESC_SIZE] = {
    // config descriptor
    sizeof(usb_configuration_descriptor_t),    // bLength
    USB_DT_CONFIG,                             // bDescriptorType
    CONFIG_DESC_SIZE, 0,                       // wTotalLength
    1,                                         // bNumInterfaces
    1,                                         // bConfigurationValue
    0,                                         // iConfiguration
    0xE0,                                      // bmAttributes = self powered
    0,                                         // bMaxPower
    // interface descriptor
    sizeof(usb_interface_descriptor_t),         // bLength
    USB_DT_INTERFACE,                           // bDescriptorType
    0,                                          // bInterfaceNumber
    0,                                          // bAlternateSetting
    1,                                          // bNumEndpoints
    USB_CLASS_HUB,                              // bInterfaceClass
    0,                                          // bInterfaceSubClass
    0,                                          // bInterfaceProtocol
    0,                                          // iInterface
    // endpoint descriptor
    sizeof(usb_endpoint_descriptor_t),          // bLength
    USB_DT_ENDPOINT,                            // bDescriptorType
    USB_ENDPOINT_IN | 1,                        // bEndpointAddress
    USB_ENDPOINT_INTERRUPT,                     // bmAttributes
    4, 0,                                       // wMaxPacketSize
    0xff,                                       // bInterval
};