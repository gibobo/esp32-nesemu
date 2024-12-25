/* vim: set tabstop=3 expandtab:
**
** This file is in the public domain.
**
** osd.c
**
** $Id: osd.c,v 1.2 2001/04/27 14:37:11 neil Exp $
**
*/

#include <errno.h>
#include <fcntl.h>
#include <limits.h>
#include <signal.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <sys/time.h>
#include <sys/stat.h>
#include <sys/types.h>
#include <unistd.h>
       
#include <noftypes.h>
#include <nofconfig.h>
#include <log.h>
#include <osd.h>
#include <nofrendo.h>

#include <version.h>

#include "nvs_flash.h"
#include "esp_partition.h"
#include "spi_flash_mmap.h"

char configfilename[]="na";

/* This is os-specific part of main() */
int osd_main(int argc, char *argv[])
{
   config.filename = configfilename;

   return main_loop("rom", system_autodetect);
}

/* File system interface */
void osd_fullname(char *fullname, const char *shortname)
{
   strncpy(fullname, shortname, PATH_MAX);
}

/* This gives filenames for storage of saves */
char *osd_newextension(char *string, char *ext)
{
   return string;
}

/* This gives filenames for storage of PCX snapshots */
int osd_makesnapname(char *filename, int len)
{
   return -1;
}

char *osd_getromdata()
{
   char *romdata;
   const esp_partition_t *part;
   spi_flash_mmap_handle_t hrom;
   esp_err_t err;
   nvs_flash_init();
   part = esp_partition_find_first(0x40, 1, NULL);
   if (part == 0)
      printf("Couldn't find rom part!\n");
   err = esp_partition_mmap(part, 0, 3 * 1024 * 1024, SPI_FLASH_MMAP_DATA, (const void **)&romdata, &hrom);
   if (err != ESP_OK)
      printf("Couldn't map rom part!\n");
   printf("Initialized. ROM@%p\n", romdata);
   return (char *)romdata;
}