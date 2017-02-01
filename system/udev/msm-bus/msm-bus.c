#include <assert.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include <ddk/device.h>
#include <ddk/driver.h>
#include <ddk/binding.h>
#include <ddk/protocol/google.h>




void devhost_launch_devhost(mx_device_t* parent, const char* name, uint32_t protocol_id,
                            const char* procname, int argc, char** argv);

static mx_status_t msm_root_init(mx_driver_t* driver) {

    char* v = getenv("magenta.fbuffer");
    if (v) {
        printf("MSMBUS: Got env string = %s\n",v);
    } else {
        printf("MSMBUS: No environment strings\n");
    }


    char name[32];
    snprintf(name, sizeof(name), "soc");

    char procname[64];
    snprintf(procname, sizeof(procname), "devhost:soc:msm");

    char arg1[20];
    snprintf(arg1, sizeof(arg1), "soc");

    char arg2[20];
    snprintf(arg2, sizeof(arg2), "%d", SOC_VID_GOOGLE);

    char arg3[20];
    snprintf(arg3, sizeof(arg3), "%d", SOC_DID_TRAPPER);

    const char* args[4] = { "/boot/bin/devhost", arg1 , arg2, arg3};
    devhost_launch_devhost(driver_get_root_device(), name, MX_PROTOCOL_SOC, procname, 4, (char**)args);

    return NO_ERROR;
}


//#ifdef TRAPPER

mx_driver_t _driver_msmroot = {
    .ops = {
        .init = msm_root_init,
    },
};

MAGENTA_DRIVER_BEGIN(_driver_msmroot, "soc", "magenta", "0.1", 0)
MAGENTA_DRIVER_END(_driver_msmroot)

//#endif