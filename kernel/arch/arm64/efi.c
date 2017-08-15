// Copyright 2016 The Fuchsia Authors
// Use of this source code is governed by a MIT-style
// license that can be found in the LICENSE file or at
// https://opensource.org/licenses/MIT

#include <assert.h>
#include <debug.h>
#include <stdlib.h>
#include <arch.h>
#include <arch/ops.h>
#include <arch/arm64.h>
#include <arch/arm64/mmu.h>
#include <arch/efi.h>
#include <arch/mp.h>
#include <bits.h>
#include <kernel/cmdline.h>
#include <kernel/thread.h>
#include <lk/init.h>
#include <lk/main.h>
#include <magenta/errors.h>
#include <inttypes.h>
#include <libfdt.h>
#include <platform.h>
#include <string.h>
#include <trace.h>

#include <pdev/driver.h>
#include <pdev/uart.h>

efi_system_table_t *sys_table = NULL;

uint64_t efi_boot(void* handle, efi_system_table_t *systable, paddr_t image_addr) __EXTERNALLY_VISIBLE;

static uint32_t efi_utf16_ascii_len(const u16 *src, int n) {
    uint32_t count = 0;
    uint16_t c;
    while (n--) {
        c = *src++;
        if (c < 0x80)
            count++;
    }
    return count;
}

static char *efi_utf16_to_ascii(char *dst, const u16 *src, int n)
{
    uint32_t c;

    while (n--) {
        c = *src++;
        if (c < 0x80) {
            *dst++ = c;
            continue;
        }
        if (c < 0x800) {
            *dst++ = 0xc0 + (c >> 6);
            goto t1;
        }
        if (c < 0x10000) {
            *dst++ = 0xe0 + (c >> 12);
            goto t2;
        }
        *dst++ = 0xf0 + (c >> 18);
        *dst++ = 0x80 + ((c >> 12) & 0x3f);
    t2:
        *dst++ = 0x80 + ((c >> 6) & 0x3f);
    t1:
        *dst++ = 0x80 + (c & 0x3f);
    }

    return dst;
}

static int efi_putc(char c)
{
    struct efi_simple_text_output_protocol *out;
    if (sys_table) {
        out = (struct efi_simple_text_output_protocol *)sys_table->con_out;
        efi_char16_t ch[2] = { 0 };
        ch[0] = c;
        out->output_string(out, ch);
    }

    return 1;
}

static int efi_getc(bool wait)
{

    return -1;
}

static int efi_pgetc(void)
{
       return -1;
}

static const struct pdev_uart_ops uart_ops = {
    .putc = efi_putc,
    .getc = efi_getc,
    .pputc = efi_putc,
    .pgetc = efi_pgetc,
};


static void efi_print(efi_system_table_t *sys_table_arg, const char *str)
{
	int i;
	struct efi_simple_text_output_protocol *out;
	out = (struct efi_simple_text_output_protocol *)sys_table_arg->con_out;

	for (i = 0; str[i]; i++) {
		efi_char16_t ch[2] = { 0 };

		ch[0] = str[i];
		if (str[i] == '\n') {
			efi_char16_t nl[2] = { '\r', 0 };
			out->output_string(out, nl);
		}
		out->output_string(out, ch);
		//efi_char16_printk(sys_table_arg, ch);
	}
}
void e_print(const char *str);
void e_print(const char *str) {
    if (sys_table) {
        efi_print(sys_table,str);
    }
}

static void efi_printhex(efi_system_table_t *systable, const char *str, void* ptr) {

	efi_print(systable,str);

	int i;
	struct efi_simple_text_output_protocol *out;
	out = (struct efi_simple_text_output_protocol *)systable->con_out;

	efi_char16_t vals[2] = {0};
	for( i=0 ; i<16; i++) {
		uint16_t val = (((uint64_t)ptr) >> ((15-i)*4)) & 0x0f;
		if (val < 10) {
			val += u'0';
		} else {
			val += (u'a' - 10);
		}
		vals[0] = val;
		out->output_string(out,vals);
	}
	efi_char16_t nl[3] = { '\r','\n', 0 };
	out->output_string(out, nl);
}

extern uint64_t _start;
extern uint64_t _end;


uint64_t efi_boot(void* handle, efi_system_table_t *systable, paddr_t image_addr) {

	efi_status_t status;
	efi_loaded_image_t *image;
	efi_guid_t loaded_image_proto = LOADED_IMAGE_PROTOCOL_GUID;

    sys_table = systable;
        pdev_register_uart(&uart_ops);


	efi_printhex(systable,"image addr :",(void*)image_addr);

	efi_print(systable,"Booting Magenta from EFI loader...\n");

	status = systable->boottime->handle_protocol(handle,
					&loaded_image_proto, (void *)&image);
	if (status != EFI_SUCCESS) {
		efi_print(systable, "Failed to get loaded image protocol\n");
		goto fail;
	}
	efi_printhex(systable,"image :",image);
    efi_printhex(systable,"image-base :",image->image_base);
    efi_printhex(systable,"image-size :",(void*)image->image_size);

	struct efi_simple_text_output_protocol *out;
	out = (struct efi_simple_text_output_protocol *)systable->con_out;
	out->output_string(out, image->load_options);
	efi_print(systable,"\n");

    char buff[256];
    snprintf(buff,256,"in length =%u    ascii=%u\n",image->load_options_size,
                                efi_utf16_ascii_len(image->load_options,image->load_options_size/2));
    efi_print(systable,buff);
/*
	struct file_info *files;
	status = systable->boottime->allocate_pool(EFI_LOADER_DATA, sizeof(*files), (void **)&files);
	if (status != EFI_SUCCESS) {
		efi_print(systable, "Failed to allocate space for file_info\n");
		goto fail;
	}
*/
	// Allocate space for new kernel location (+bss)
	uint64_t kern_pages = (uint64_t)&_end - (uint64_t)&_start;
	kern_pages = ROUNDUP(kern_pages, EFI_ALLOC_ALIGN) / EFI_PAGE_SIZE;
	//efi_printhex(systable,"_start :",(void*)&_start);
	//efi_printhex(systable,"_end :",(void*)&_end);
	//efi_printhex(systable,"kern_pages :",(void*)kern_pages);
	efi_physical_addr_t target_addr = MEMBASE + KERNEL_LOAD_OFFSET;
	status = systable->boottime->allocate_pages( EFI_ALLOCATE_ADDRESS,
	                                             EFI_LOADER_DATA,
	                                             kern_pages,
	                                             &target_addr);
	if (status != EFI_SUCCESS) {
		efi_print(systable, "Failed to allocate space for kernel\n");
		goto fail;
	}

	// Copy kernel to new location
	memcpy((void*)target_addr,(void*)image_addr,kern_pages*EFI_PAGE_SIZE);




	//snprintf(buff,256,"target addr =0x%lx\n",target_addr);
	//efi_print(systable,buff);

	//uint32_t *checkup = (uint32_t*)target_addr;
	//for (int i = 0; i<20; i++)
	//	efi_printhex(systable,"mem:",(void*)(uint64_t)checkup[i]);



	// find the fdt in image (does it need to be relocated?)
	//uint16_t tempchars[2] = {0};
    /*
	efi_guid_t fdt_guid = DEVICE_TREE_GUID;
	efi_config_table_t *tables = (efi_config_table_t *) systable->tables;
	uint32_t *fdt=0;
	for (uint i=0; i <  systable->nr_tables; i++) {
		if (!memcmp((char*)(&tables[i].guid), (char*)&fdt_guid,16 )) {
			fdt = (uint32_t*)tables[i].table;
			efi_print(systable,"found fdt");
		}
		for (int j=0; j<16; j++) {
			snprintf(buff,256,"%02x",((uint8_t*)(&tables[i].guid))[j]);
			efi_print(systable,buff);
			//out->output_string(out,tempchars);
		}
		efi_print(systable,"\n");
	}

	int offset = fdt_path_offset(fdt, "/chosen");
    if (offset < 0) {
        efi_print(systable,"couldn't parse fdt\n");
        return 0;
    }

    int length;
    const char* bootargs = fdt_getprop(fdt,offset, "bootargs",&length);
    if (bootargs) {
        efi_print(systable,bootargs);
        efi_print(systable,"\n");

    } else {
        efi_print(systable,"Could not find bootargs\n");
    }
*/
    //allocate one page for the magenta boot args
    //efi_physical_addr_t args_addr;
    efi_magenta_hdr_t *mag_hdr;

    uint32_t cmd_line_len = efi_utf16_ascii_len(image->load_options,image->load_options_size/2) + 1;

    status = systable->boottime->allocate_pool(EFI_LOADER_DATA, sizeof(*mag_hdr) + cmd_line_len,
                                                                (void **)&mag_hdr);
    if (status != EFI_SUCCESS) {
        efi_print(systable, "Failed to allocate space for magenta boot args\n");
        goto fail;
    }

    snprintf(buff,256,"Magenta boot args address= %p\n",(void*)mag_hdr);
    efi_print(systable,buff);

    mag_hdr->magic = EFI_MAGENTA_MAGIC;
    mag_hdr->cmd_line_len = cmd_line_len;
    efi_utf16_to_ascii(mag_hdr->cmd_line, image->load_options, image->load_options_size/2);
    mag_hdr->cmd_line[cmd_line_len-1]=0;

    snprintf(buff,256,"Magenta cmdline args = %s\n",mag_hdr->cmd_line);
    efi_print(systable,buff);
    const char token[] = "initrd=";
    char* pos;
    uint64_t initrd_start_phys=0;
    uint64_t initrd_size=0;
    pos = strstr(mag_hdr->cmd_line,token);
    if (pos) {
        pos = pos + strlen(token);
        initrd_start_phys = strtoll(pos,&pos,16);
        pos++;
        initrd_size = strtoll(pos,&pos,16);
    }
    efi_printhex(systable,"initrd-start",(void*)initrd_start_phys);
    efi_printhex(systable,"initrd-size",(void*)initrd_size);
    //cmdline_append(mag_hdr->cmd_line);
    //cmdline_append("initrd=56,44");
    //e_print("cmdline appended\n");
/*
    uint64_t initrd_start_phys=0;
    uint64_t initrd_size=0;
    const char* value = cmdline_get("initrd");
    e_print("did get\n");

    if (value != NULL) {
        char* endptr;
        initrd_start_phys = strtoll(value,&endptr,16);
        endptr++; //skip the comma
        initrd_size = strtoll(endptr,NULL,16);
    }
*/
    if (initrd_start_phys && initrd_size) {

        arch_sync_cache_range((addr_t)initrd_start_phys,initrd_size);
        e_print("initrd found and flushed from cache...\n");
    } else {
        e_print("initrd not found!!!!!\n");
    }


    // sync cache (we jumped here with mmu on w/ identity and cache on)
    arch_sync_cache_range((addr_t)target_addr, kern_pages*EFI_PAGE_SIZE);
    arch_sync_cache_range((addr_t)mag_hdr, sizeof(*mag_hdr) + cmd_line_len);
/*
    for (uint64_t i=initrd_start_phys; i < (initrd_start_phys+initrd_size); i=i+1024) {
        snprintf(buff,256,"%08lx  -  %016lx\n",i,*(uint64_t*)(i));
        efi_print(systable,buff);
    }
*/

    e_print("yo dawg, here we go\n");

	return (uint64_t)mag_hdr;

	// shut down mmu/cache

	// jump to hyperspace


	//return 0xb00beee5;

fail:
	return 0xdeadbeef;

}