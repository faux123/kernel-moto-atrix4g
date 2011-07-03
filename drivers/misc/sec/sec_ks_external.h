/*
 * Copyright (C) 2009 Motorola, Inc.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA  02111-1307  USA
 */

/*
 * Date         Author          Comment
 * ===========  ==============  ==============================================
 * 16-Apr-2009  Motorola        Initial revision.
 *
 */

/*Header for external kernel space SecAPIs*/

#define SEC_PROD_ID_SIZE	16
#define SEC_MODEL_ID_SIZE	4
#define SEC_SERNO_SIZE		10
#define SEC_PROC_UID_SIZE	16
#define SEC_SBK_SIZE    	16

/* If caller is user space app, SEC_BASE_MODE is substructed before reporting SEC_MODE back to comply with 
	SEC_IC_TYPE_xxx values defined in SecPublic.h */
typedef enum {
	SEC_MODE_BASE	 = 0x00000077, 
	SEC_ENGINEERING, // 0x0078 - 0x0077 = 0x0001
	SEC_PRODUCTION,	 // 0x0079 - 0x0077 = 0x0002
	SEC_SEC_DISABLED = 0x0000007f, //-0x0077 = 0x0008 
    /** Ignore -- Forces compilers to make 16-bit enums. */
	SEC_Force32      = 0x7FFFFFFF
} SEC_MODE_T;


typedef enum {
	SEC_SUCCESS = 0x33,
	SEC_FAIL = 0x00
} SEC_STAT_T;

typedef enum {
	SEC_FLAG_ENGINEERING = 0x00000001,
	SEC_FLAG_PRODUCTION  = 0x00000002,
	SEC_FLAG_SECURE      = 0x00000004,
	SEC_FLAG_INFACTORY   = 0x00000008,
    SEC_FLAG_Force32     = 0x7FFFFFFF
} SEC_FLAGS_T;


SEC_STAT_T SecProcessorUID (unsigned char *buffer, int length);
SEC_STAT_T SecModelID (unsigned char *buffer, int length);
SEC_STAT_T SecProductID (unsigned char *buffer, int length);
SEC_STAT_T SecSerialID (unsigned char *buffer, int length);

int SecIsUnitSecure (void);
int SecIsUnitProduction (void);
int SecIsUnitEngineering (void);
int SecUnitGetFlags (unsigned long *flags);

SEC_MODE_T SecProcessorType (void);
