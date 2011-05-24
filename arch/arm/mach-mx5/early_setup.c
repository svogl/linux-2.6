/*
 * Copyright (C) 2010 Freescale Semiconductor, Inc. All Rights Reserved.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.

 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.

 * You should have received a copy of the GNU General Public License along
 * with this program; if not, write to the Free Software Foundation, Inc.,
 * 51 Franklin Street, Fifth Floor, Boston, MA 02110-1301 USA.
 */

#include <linux/init.h>
#include <linux/string.h>

int __initdata primary_di = { 0 };
static int __init di_setup(char *__unused)
{
	primary_di = 1;
	return 1;
}
__setup("di1_primary", di_setup);

int __initdata enable_tvout = { 0 };
static int __init tvout_setup(char *s)
{
        /* PAL is default */

        if (strcmp(s, "=ntsc") == 0)
                enable_tvout = 2; /* ntsc */
        else
                enable_tvout = 1; /* pal */
        return 1;
}
__setup("tvout", tvout_setup);

int __initdata enable_emerging_display = { 0 };
int __initdata enable_toshiba_display = { 0 };
char __initdata enable_hdmi[20];
static int __init display_setup(char *s)
{
        int len;
        enable_hdmi[0] = '\0';
        if (strcmp(s, "=emerging") == 0)
                enable_emerging_display = 1;
        else if (strcmp(s, "=toshiba") == 0)
                enable_toshiba_display = 1;
        else if (strncmp(s, "=hdmi:", 6) == 0) {
                len = strlen(s) - 6;
                len = len > 19 ? 19 : len;
                strncpy(enable_hdmi, s+6, len);
        }
        return 1;
}
__setup("display", display_setup);

