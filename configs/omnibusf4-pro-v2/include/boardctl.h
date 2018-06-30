/*
 * boardctl.h
 *
 *  Created on: 30 июн. 2018 г.
 *      Author: developer
 */

#ifndef NUTTX_CONFIGS_OMNIBUSF4_PRO_V2_INCLUDE_BOARDCTL_H_
#define NUTTX_CONFIGS_OMNIBUSF4_PRO_V2_INCLUDE_BOARDCTL_H_

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <sys/boardctl.h>

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#define GRANUM_FIRE_CHUTE_DEPLOY	BOARDIOC_USER+1
#define GRANUM_FIRE_CHUTE_CUT		BOARDIOC_USER+2
#define GRANUM_FIRE_LEGS_DEPLOY		BOARDIOC_USER+3

#endif /* NUTTX_CONFIGS_OMNIBUSF4_PRO_V2_INCLUDE_BOARDCTL_H_ */
