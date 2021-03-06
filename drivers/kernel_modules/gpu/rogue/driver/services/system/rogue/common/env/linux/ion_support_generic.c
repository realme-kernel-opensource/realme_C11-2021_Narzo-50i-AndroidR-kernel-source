/*************************************************************************/ /*!
@File           ion_support_generic.c
@Title          Generic Ion support
@Copyright      Copyright (c) Imagination Technologies Ltd. All Rights Reserved
@Description    This file does the Ion initialisation and De-initialisation for
                systems that don't already have Ion.
                For systems that do have Ion it's expected they init Ion as
                per their requirements and then implement IonDevAcquire and
                IonDevRelease which provides access to the ion device.
@License        Dual MIT/GPLv2

The contents of this file are subject to the MIT license as set out below.

Permission is hereby granted, free of charge, to any person obtaining a copy
of this software and associated documentation files (the "Software"), to deal
in the Software without restriction, including without limitation the rights
to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
copies of the Software, and to permit persons to whom the Software is
furnished to do so, subject to the following conditions:

The above copyright notice and this permission notice shall be included in
all copies or substantial portions of the Software.

Alternatively, the contents of this file may be used under the terms of
the GNU General Public License Version 2 ("GPL") in which case the provisions
of GPL are applicable instead of those above.

If you wish to allow use of your version of this file only under the terms of
GPL, and not to allow others to use your version of this file under the terms
of the MIT license, indicate your decision by deleting the provisions above
and replace them with the notice and other provisions required by GPL as set
out in the file called "GPL-COPYING" included in this distribution. If you do
not delete the provisions above, a recipient may use your version of this file
under the terms of either the MIT license or GPL.

This License is also included in this distribution in the file called
"MIT-COPYING".

EXCEPT AS OTHERWISE STATED IN A NEGOTIATED AGREEMENT: (A) THE SOFTWARE IS
PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR IMPLIED, INCLUDING
BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY, FITNESS FOR A PARTICULAR
PURPOSE AND NONINFRINGEMENT; AND (B) IN NO EVENT SHALL THE AUTHORS OR
COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY, WHETHER
IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN
CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.
*/ /**************************************************************************/

#include <linux/version.h>

#include "pvrsrv_error.h"
#include "ion_support.h"

#if (LINUX_VERSION_CODE < KERNEL_VERSION(4, 12, 0))
#include "img_types.h"
#include "pvr_debug.h"
#include "ion_sys.h"

#ifdef GRALLOC_ARM_DMA_BUF_MODULE
#include <linux/ion.h>
#include <ion/ion.h>
#endif
//#include "ion_sprd.h"
//#include <Ion_priv.h>

//Ion_priv.h (kernel\drivers\staging\android\ion):void ion_device_destroy(struct ion_device *dev);

#include "pvrsrv_error.h"
#include "img_types.h"
#include "pvr_debug.h"
#include "ion_support.h"
#include "ion_sys.h"

extern struct ion_device *idev;

PVRSRV_ERROR IonInit(void *pvPrivateData)
{
	/*Nothing to do */
	return PVRSRV_OK;
}

struct ion_device *IonDevAcquire(void)
{
	return idev;
}

void IonDevRelease(struct ion_device *psIonDev)
{
	/* Nothing to do, sanity check the pointer we're passed back */
	PVR_ASSERT(psIonDev == idev);
}

void IonDeinit(void)
{
	/*nothing to do*/
}
#endif	/* (LINUX_VERSION_CODE < KERNEL_VERSION(4, 12, 0)) */
