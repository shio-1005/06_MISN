/* 
 * File:   MISN.h
 * Author: shiohitomi
 *
 * Created on November 5, 2021
 */

#ifndef MISN_H
#define	MISN_H

/*--Message ID--*/
/* SIDH */
#define SIDH_READ       0b00000110
#define SIDH_1          0b00000001
#define SIDH_2          0b00000010

/* SIDL */
#define SIDL_READ       0b00001000
#define SIDL_DATA1      0b00001001                                              //モード送信
#define SIDL_DATA2      0b00101001
#define SIDL_DATA3      0b01001001
#define SIDL_DATA4      0b01101001
#define SIDL_DATA5      0b10001001
#define SIDL_DATA6      0b10101001
#define SIDL_DATA7      0b11001001
#define SIDL_DATA8      0b11101001
#define SIDL_DATA9      0b00001001
#define SIDL_DATA10     0b00101001

/* EID8 */
#define EID8_WRITE      0b00000110

/* EID0 */
#define EID0_DATA1      0b00000000                                              //モード送信
#define EID0_DATA2      0b00001000
#define EID0_DATA3      0b00010000
#define EID0_DATA4      0b00011000
#define EID0_DATA5      0b00100000
#define EID0_DATA6      0b00101000
#define EID0_DATA7      0b00110000
#define EID0_DATA8      0b00111000
#define EID0_DATA9      0b01000000
#define EID0_DATA10     0b01001000

#endif
