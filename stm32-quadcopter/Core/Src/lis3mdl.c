/*
 * lis3mdl.c
 *
 *  Created on: Sep 20, 2025
 *      Author: prate
 */

#include "lis3mdl.h"
#include "i2c.h"
#include "stm32f4xx.h"
#include "stm32f446xx.h"

#include <math.h>
#include <stdbool.h>
#include <stdint.h>

//CTRL_REGX
#define TEMP_EN  (1U << 7)
#define OM(mode) (mode << 5)
#define OM_MASK  (3U << 5)
#define DO(mode) (mode << 2)
#define DO_MASK  (7U << 2)
#define FAST_ODR (1U << 1)
#define ST       (1U << 0)

#define FS(mode) (mode << 5)
#define FS_MASK  (3U << 5)
#define REBOOT   (1U << 3)
#define SOFT_RST (1U << 2)

#define LP       (1U << 5)
#define SIM      (1U << 2)
#define MD(mode) (mode << 0)
#define MD_MASK  (3U << 0)

#define OMZ(mode) (mode << 2)
#define OMZ_MASK  (3U << 2)
#define BLE       (1U << 1)

#define FAST_READ (1U << 7)
#define BDU       (1U << 6)

#define ZYXOR (1U << 7)
#define ZOR   (1U << 6)
#define YOR   (1U << 5)
#define XOR   (1U << 4)
#define ZYXDA (1U << 3) //IMPORTANT
#define ZDA   (1U << 2)
#define YDA   (1U << 1)
#define XDA   (1U << 0)

#define XIEN (1U << 7)
#define YIEN (1U << 6)
#define ZIEN (1U << 5)
#define IEA  (1U << 2)
#define LIR  (1U << 1)
#define IEN  (1U << 0)

#define PTH_X (1U << 7)
#define PTH_Y (1U << 6)
#define PTH_Z (1U << 5)
#define NTH_X (1U << 4)
#define NTH_Y (1U << 3)
#define NTH_Z (1U << 2)
#define MROI  (1U << 1)
#define INT   (1U << 0)

//Function Prototypes
