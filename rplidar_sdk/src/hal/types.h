/*
 *      Common Data Types for RP
 */

#ifndef _INFRA_HAL_TYPES_H_
#define _INFRA_HAL_TYPES_H_

#include "../../include/rptypes.h"  // Shared types used also for interface

//#define __le
//#define __be

#define _multi_thread
#define _single_thread

typedef uint32_t u_result;

// See other RESULT_XXX-definitions at "../../include/rptypes.h"
// (These weren't originally defined there, hopefully not returned outside...)
#define RESULT_REMAINING_DATA           0x21
#define RESULT_OPERATION_ABORTED        (0x8007 | RESULT_FAIL_BIT)
#define RESULT_NOT_FOUND                (0x8008 | RESULT_FAIL_BIT)
#define RESULT_RECONNECTING             (0x8009 | RESULT_FAIL_BIT)

#endif
